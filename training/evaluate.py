import argparse
import json
from pathlib import Path

from t3_policy import WalkingPolicy
from training.local_env import LocalGo1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('model_dir', type=Path)
    parser.add_argument('checkpoint', type=Path)
    parser.add_argument('--speed', type=float, default=0.4)
    parser.add_argument('--frequency', type=float, default=3.0)
    parser.add_argument('--kp', type=float, default=20.0)
    parser.add_argument('--kd', type=float, default=0.5)
    parser.add_argument('--swing-height', type=float, default=0.08)
    parser.add_argument('--stance-width', type=float, default=0.25)
    parser.add_argument('--hold-seconds', type=float, default=0.3)
    parser.add_argument('--time-limit', type=float, default=90.0)
    parser.add_argument('--episodes', type=int, default=1)
    parser.add_argument('--seed', type=int)
    args = parser.parse_args()
    policy = WalkingPolicy(args.checkpoint)
    env = LocalGo1(args.model_dir, args.speed, args.frequency, args.kp, args.kd,
                   args.swing_height, args.stance_width, args.hold_seconds)
    for episode in range(args.episodes):
        seed = None if args.seed is None else args.seed + episode
        history = env.reset(seed)
        while not env.fell and env.distance < 3.0 and env.data.time < 1.5 + args.time_limit:
            history = env.step(policy.action(history))
        print(json.dumps({
            'episode': episode + 1, 'seed': seed, 'speed_mps': args.speed,
            'sim_seconds': round(env.data.time, 3), 'forward_m': round(env.distance, 3),
            'fell': env.fell, 'success': not env.fell and env.distance >= 3.0,
        }), flush=True)


if __name__ == '__main__':
    main()
