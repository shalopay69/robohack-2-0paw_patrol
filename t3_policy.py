import math

import numpy as np


POLICY_PERIOD = 0.02
UNITREE_TO_POLICY = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8])
DEFAULT_POSITION = np.array(
    [0.1, 0.8, -1.5, -0.1, 0.8, -1.5, 0.1, 1.0, -1.5, -0.1, 1.0, -1.5],
    dtype=np.float32,
)
ACTION_SCALE = np.tile(np.array([0.125, 0.25, 0.25], dtype=np.float32), 4)


def joint_targets(action):
    return (DEFAULT_POSITION + ACTION_SCALE * np.clip(action, -10.0, 10.0))[UNITREE_TO_POLICY]


class ObservationHistory:
    def __init__(self, frequency=3.0, swing_height=0.08, stance_width=0.25):
        self.frequency = frequency
        self.gait = np.array([0, frequency, 0.5, 0, 0, 0.5, swing_height * 0.15, 0, 0,
                              stance_width, 0, 0], dtype=np.float32)
        self.frames = np.zeros((30, 70), dtype=np.float32)
        self.last_action = np.zeros(12, dtype=np.float32)
        self.phase = 0.0

    def append(self, q, dq, quaternion, velocity, action):
        w, x, y, z = np.asarray(quaternion) / np.linalg.norm(quaternion)
        gravity = [-2 * (x * z - w * y), -2 * (y * z + w * x), 2 * (x * x + y * y) - 1]
        commands = np.asarray(velocity) * [2.0, 2.0, 0.25]
        clock = np.sin(math.tau * (self.phase + np.array([0.5, 0, 0, 0.5])))
        self.frames[:-1] = self.frames[1:]
        self.frames[-1] = np.concatenate(
            (gravity, commands, self.gait, np.asarray(q)[UNITREE_TO_POLICY] - DEFAULT_POSITION,
             np.asarray(dq)[UNITREE_TO_POLICY] * 0.05, action, self.last_action, clock)
        )
        self.last_action = np.asarray(action, dtype=np.float32).copy()
        self.phase = (self.phase + self.frequency * POLICY_PERIOD) % 1.0
        return self.frames.ravel()


class WalkingPolicy:
    def __init__(self, checkpoint):
        with np.load(checkpoint, allow_pickle=False) as arrays:
            self.adaptation = self.load_layers(arrays, 'adaptation', (2100, 256, 128, 2))
            self.body = self.load_layers(arrays, 'body', (2102, 512, 256, 128, 12))

    @staticmethod
    def load_layers(arrays, name, sizes):
        layers = []
        for index, (inputs, outputs) in enumerate(zip(sizes, sizes[1:])):
            w = arrays[f'{name}_{index}_w']
            b = arrays[f'{name}_{index}_b']
            if w.shape != (outputs, inputs) or b.shape != (outputs,):
                raise ValueError(f'Unexpected shape for {name} layer {index}')
            if not np.isfinite(w).all() or not np.isfinite(b).all():
                raise ValueError(f'Nonfinite weights in {name} layer {index}')
            layers.append((w.astype(np.float32), b.astype(np.float32)))
        return layers

    @staticmethod
    def forward(values, layers, activate_last=False):
        for index, (weight, bias) in enumerate(layers):
            values = weight @ values + bias
            if activate_last or index < len(layers) - 1:
                values = np.where(values > 0, values, np.expm1(np.minimum(values, 0.0)))
        return values

    def features(self, history):
        latent = self.forward(history, self.adaptation)
        return self.forward(np.concatenate((history, latent)), self.body[:-1], activate_last=True)

    def action(self, history):
        weight, bias = self.body[-1]
        return weight @ self.features(history) + bias
