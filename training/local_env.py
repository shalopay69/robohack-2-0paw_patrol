"""Headless MuJoCo rollouts with the same motor law and Safety Block as the ROS run."""

import math
from pathlib import Path

import mujoco
import numpy as np

from t2_safety import CollisionChecker, SafetyBlock
from t3_policy import POLICY_PERIOD, ObservationHistory, joint_targets


TORQUE_LIMIT = np.tile([23.7, 23.7, 35.55], 4)


class LocalGo1:
    def __init__(self, model_dir, speed, frequency=3.0, kp=20.0, kd=0.5,
                 swing_height=0.08, stance_width=0.25, hold_seconds=0.3):
        self.model = mujoco.MjModel.from_xml_path(str(Path(model_dir) / 'scene.xml'))
        self.data = mujoco.MjData(self.model)
        self.checker = CollisionChecker(model_dir)
        self.q = self.checker.position_indices
        self.dq = self.model.jnt_dofadr[
            [self.model.joint(f'{leg}_{joint}_joint').id
             for leg in ('FR', 'FL', 'RR', 'RL') for joint in ('hip', 'thigh', 'calf')]
        ]
        self.model.actuator_gainprm[:, :] = 0.0
        self.model.actuator_biasprm[:, :] = 0.0
        self.speed = speed
        self.frequency = frequency
        self.swing_height = swing_height
        self.stance_width = stance_width
        self.hold_seconds = hold_seconds
        self.command = np.zeros((12, 5))
        self.command[:, 3:] = [kp, kd]

    def reset(self, seed=None):
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key('home').id)
        if seed is not None:
            rng = np.random.default_rng(seed)
            self.data.qpos[self.q] += rng.normal(0.0, 0.008, 12)
            self.data.qvel[self.dq] += rng.normal(0.0, 0.03, 12)
        mujoco.mj_forward(self.model, self.data)
        self.safety = SafetyBlock(self.checker)
        self.history = ObservationHistory(self.frequency, self.swing_height, self.stance_width)
        self.next_safety = 0.0
        self.control_index = 0
        self.active = self.safety.active
        self.fall_timers = np.zeros(3)
        self.fell = False
        self.start_x = None
        while self.data.time < self.hold_seconds:
            self.step_physics(np.column_stack(
                (self.checker.home, np.tile([0.0, 0.0, 40.0, 2.0], (12, 1)))
            ))
        self.first_policy_time = self.data.time
        return self.observe(np.zeros(12, dtype=np.float32))

    def observe(self, previous_action):
        return self.history.append(
            self.data.qpos[self.q], self.data.qvel[self.dq], self.data.qpos[3:7],
            (self.speed, 0.0, 0.0), previous_action,
        )

    def step_physics(self, command):
        q, dq, tau, kp, kd = command.T
        torque = tau + kp * (q - self.data.qpos[self.q]) + kd * (dq - self.data.qvel[self.dq])
        self.data.qfrc_applied[self.dq] = np.clip(torque, -TORQUE_LIMIT, TORQUE_LIMIT)
        mujoco.mj_step(self.model, self.data)
        w, x, y, z = self.data.qpos[3:7]
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = math.asin(float(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
        violations = [self.data.qpos[2] < 0.16, abs(roll) > math.radians(50),
                      abs(pitch) > math.radians(50)]
        self.fall_timers = np.where(violations, self.fall_timers + self.model.opt.timestep, 0.0)
        self.fell |= bool(np.any(self.fall_timers > 0.5 + 1e-9))
        if self.start_x is None and self.data.time >= 1.5 - 1e-9:
            self.start_x = float(self.data.qpos[0])

    def step(self, action):
        action = np.clip(action, -10.0, 10.0)
        self.command[:, 0] = joint_targets(action)
        self.control_index += 1
        end_time = self.first_policy_time + self.control_index * POLICY_PERIOD
        while self.data.time < end_time - 1e-9:
            if self.data.time >= self.next_safety - 1e-9:
                stamp = round(self.data.time * 1e9)
                self.safety.receive(self.command, stamp)
                self.active = self.safety.sample(stamp)
                self.next_safety = self.data.time + 0.01
            self.step_physics(self.active)
        return self.observe(action)

    @property
    def distance(self):
        return 0.0 if self.start_x is None else float(self.data.qpos[0] - self.start_x)
