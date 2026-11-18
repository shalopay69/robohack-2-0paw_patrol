from itertools import pairwise
from time import monotonic

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from unitree_go.msg import LowCmd


NS = 1_000_000_000


class StreamProbe(Node):
    def __init__(self):
        super().__init__('t2_stream_probe')
        self.sim_ns = 0
        self.samples = []
        self.create_subscription(Clock, '/clock', self.update_clock, qos_profile_sensor_data)
        self.create_subscription(LowCmd, '/lowcmd', self.receive, qos_profile_sensor_data)
        self.publisher = self.create_publisher(LowCmd, '/lowcmd_bare', 1)

    def update_clock(self, message):
        self.sim_ns = message.clock.sec * NS + message.clock.nanosec

    def receive(self, message):
        if self.sim_ns:
            motors = message.motor_cmd[:12]
            assert all(motor.mode == 10 for motor in motors)
            q = np.array([motor.q for motor in motors])
            dq = np.array([motor.dq for motor in motors])
            tau = np.array([motor.tau for motor in motors])
            assert np.isfinite([q, dq, tau]).all()
            self.samples.append((self.sim_ns, q, dq, tau))


def make_command(target, dq=0.0, tau=0.0):
    message = LowCmd()
    for motor, q in zip(message.motor_cmd, target):
        motor.mode = 10
        motor.q = float(q)
        motor.kp = 20.0
        motor.kd = 1.0
    message.motor_cmd[1].dq = dq
    message.motor_cmd[1].tau = tau
    return message


def check_rate(probe, start, end):
    samples = [sample for sample in probe.samples if start < sample[0] <= end]
    stamps = [start] + [sample[0] for sample in samples] + [end]
    rate = len(samples) / ((end - start) / NS)
    max_gap = max(b - a for a, b in pairwise(stamps)) / NS
    assert rate >= 50, rate
    assert max_gap <= 0.1, max_gap
    print(f'{len(samples)} commands in 5 s sim time; {rate:.1f} Hz; max gap {max_gap * 1000:.1f} ms')
    return samples


rclpy.init()
probe = StreamProbe()
try:
    deadline = monotonic() + 20
    while not probe.samples or not probe.publisher.get_subscription_count():
        assert monotonic() < deadline, 'Timed out waiting for sim clock and Safety Block'
        rclpy.spin_once(probe, timeout_sec=0.1)

    initial_q = probe.samples[-1][1]
    upper = initial_q.copy()
    upper[1] += np.deg2rad(4.99)
    commands = (make_command(upper), make_command(initial_q))
    start = probe.sim_ns
    end = start + 5 * NS
    last_sent = 0
    sent = 0
    deadline = monotonic() + 20
    while probe.sim_ns < end:
        assert monotonic() < deadline, 'Simulation clock did not advance during stress check'
        if probe.sim_ns - last_sent >= 12_000_000:
            probe.publisher.publish(commands[sent % 2])
            last_sent = probe.sim_ns
            sent += 1
        rclpy.spin_once(probe, timeout_sec=0.05)
    print('Alternating targets 4.99 degrees apart:')
    stress = check_rate(probe, start, end)
    substantial_moves = sum(abs(after[1] - before[1]) >= 0.08
                            for (_, before, _, _), (_, after, _, _) in pairwise(stress))
    assert substantial_moves >= 150, substantial_moves
    print(f'{substantial_moves} substantial target changes accepted')

    target = probe.samples[-1][1].copy()
    target[1] -= 0.02
    message = make_command(target, dq=0.1, tau=0.2)

    start = probe.sim_ns
    end = start + 5 * NS
    last_sent = 0
    deadline = monotonic() + 20
    while probe.sim_ns < end:
        assert monotonic() < deadline, 'Simulation clock did not advance 5 seconds'
        if probe.sim_ns < start + NS and probe.sim_ns - last_sent >= 25_000_000:
            probe.publisher.publish(message)
            last_sent = probe.sim_ns
        rclpy.spin_once(probe, timeout_sec=0.05)

    print('Input dropout:')
    samples = check_rate(probe, start, end)
    assert any(np.allclose(q, target, atol=1e-6) and dq[1] > 0.09 and tau[1] > 0.19
               for _, q, dq, tau in samples), 'Safe command was not accepted'
    held = [sample for sample in samples if sample[0] >= last_sent + 500_000_000]
    assert held, 'No samples after watchdog deadline'
    for _, q, dq, tau in held:
        np.testing.assert_allclose(q, target, atol=1e-6)
        np.testing.assert_array_equal(dq, 0.0)
        np.testing.assert_array_equal(tau, 0.0)
    for (_, before, _, _), (_, after, _, _) in pairwise(samples):
        assert np.max(np.abs(after - before)) <= np.deg2rad(5.0)
    first_hold_ns = next(stamp for stamp, q, dq, tau in samples
                         if stamp > last_sent and np.allclose(q, target, atol=1e-6)
                         and not np.any(dq) and not np.any(tau))
    hold_delay = (first_hold_ns - last_sent) / NS
    assert hold_delay <= 0.5, hold_delay
    print(f'Safe setpoint accepted; watchdog held the same q after {hold_delay:.3f} s with dq = tau = 0')
finally:
    probe.destroy_node()
    rclpy.try_shutdown()
