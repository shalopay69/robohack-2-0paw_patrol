"""Convert the published Walk These Ways TorchScript checkpoint to NumPy."""

import argparse
from pathlib import Path

import numpy as np
import torch

from t3_policy import WalkingPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('upstream', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    torch.set_num_threads(1)
    modules = {
        'adaptation': torch.jit.load(
            str(args.upstream / 'adaptation_module_latest.jit'), map_location='cpu'
        ).eval(),
        'body': torch.jit.load(str(args.upstream / 'body_latest.jit'), map_location='cpu').eval(),
    }
    arrays = {}
    for name, module in modules.items():
        for key, tensor in module.state_dict().items():
            layer, field = key.split('.')
            suffix = {'weight': 'w', 'bias': 'b'}[field]
            arrays[f'{name}_{int(layer) // 2}_{suffix}'] = tensor.detach().numpy()
    np.savez_compressed(args.output, **arrays)
    runtime = WalkingPolicy(args.output)
    rng = np.random.default_rng(42)
    with torch.inference_mode():
        for _ in range(10):
            history = rng.normal(0, 0.5, size=2100).astype(np.float32)
            state = torch.from_numpy(history)
            expected = modules['body'](torch.cat((state, modules['adaptation'](state))))
            np.testing.assert_allclose(runtime.action(history), expected.numpy(), rtol=1e-4, atol=1e-5)
    print(f'{args.output}: output matches TorchScript on 10 inputs')


if __name__ == '__main__':
    main()
