import csv
import math
from pathlib import Path

import mujoco
import numpy as np


JOINT_COUNT = 12
MAX_POSITION_STEP = math.radians(5.0) - 1e-6
COLLISION_STEP = math.radians(0.1)
PUBLISH_PERIOD_NS = 10_000_000
# Reserve two publication periods so the hold reaches /lowcmd by 0.5 s.
WATCHDOG_NS = 500_000_000 - 2 * PUBLISH_PERIOD_NS
HOLD_KP = 20.0
HOLD_KD = 1.0


class CollisionChecker:
    def __init__(self, model_dir):
        model_dir = Path(model_dir)
        self.model = mujoco.MjModel.from_xml_path(str(model_dir / 'scene.xml'))
        self.data = mujoco.MjData(self.model)
        joints = [self.model.joint(f'{leg}_{joint}_joint').id
                  for leg in ('FR', 'FL', 'RR', 'RL')
                  for joint in ('hip', 'thigh', 'calf')]
        self.position_indices = self.model.jnt_qposadr[joints]
        self.lower, self.upper = self.model.jnt_range[joints].T

        with (model_dir / 't2_collision_pairs.csv').open(newline='') as pair_file:
            self.checked_pairs = {
                (int(row['geom1_id']), int(row['geom2_id']))
                for row in csv.DictReader(pair_file)
                if row['diagnostic_policy'] == 'checked'
            }
        if self.model.ngeom != 56 or len(self.checked_pairs) != 655:
            raise ValueError('Unexpected model or collision pair list for T2')

        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key('home').id)
        self.home = self.data.qpos[self.position_indices].astype(np.float32).astype(float)
        if not self.is_transition_safe(self.home, self.home):
            raise ValueError('The initial hold position has a forbidden self-collision')

    def is_transition_safe(self, start, target):
        largest_change = float(np.max(np.abs(target - start)))
        intervals = max(1, math.ceil(largest_change / COLLISION_STEP))
        for position in np.linspace(start, target, intervals + 1):
            self.data.qpos[self.position_indices] = position
            mujoco.mj_fwdPosition(self.model, self.data)
            for contact in self.data.contact[:self.data.ncon]:
                if contact.dist >= -0.001:
                    continue
                pair = tuple(sorted((contact.geom1, contact.geom2)))
                if pair in self.checked_pairs:
                    return False
        return True


class SafetyBlock:
    def __init__(self, checker):
        self.checker = checker
        self.active = np.zeros((JOINT_COUNT, 5))
        self.active[:, 0] = checker.home
        self.active = self.hold()
        self.pending = None
        self.last_input_ns = None

    def hold(self):
        command = self.active.copy()
        command[:, 1:3] = 0.0
        command[:, 3] = HOLD_KP
        command[:, 4] = HOLD_KD
        return command

    def receive(self, command, stamp_ns):
        self.last_input_ns = stamp_ns
        if not np.all(np.isfinite(command)) or np.any(command[:JOINT_COUNT, 3:] < 0):
            self.pending = None
            return
        self.pending = command[:JOINT_COUNT].copy()

    def sample(self, stamp_ns):
        if self.last_input_ns is None or not 0 <= stamp_ns - self.last_input_ns < WATCHDOG_NS:
            self.pending = None
            self.active = self.hold()
        elif self.pending is not None:
            command = self.pending
            self.pending = None
            previous = self.active[:, 0]
            requested = command[:, 0].copy()
            target = np.clip(requested, self.checker.lower + 1e-6, self.checker.upper - 1e-6)
            target = previous + np.clip(target - previous, -MAX_POSITION_STEP, MAX_POSITION_STEP)
            target = target.astype(np.float32).astype(float)
            if self.checker.is_transition_safe(previous, target):
                command[:, 0] = target
                corrected = np.abs(requested - target) > 1e-7
                command[corrected, 1:3] = 0.0
                self.active = command
            else:
                self.active = self.hold()
        return self.active
