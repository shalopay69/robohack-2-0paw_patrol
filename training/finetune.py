"""Fine-tune the pretrained actor's output layer with PPO in MuJoCo."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.distributions import Independent, Normal

from t3_policy import WalkingPolicy
from training.local_env import LocalGo1


def reward(env):
    vx, vy = env.data.qvel[:2]
    w, _, _, z = env.data.qpos[3:7]
    yaw_cost = min(1.0, abs(2.0 * np.arctan2(z, w)))
    tracking = np.exp(-((vx - env.speed) / 0.36) ** 2)
    stability = np.clip((env.data.qpos[2] - 0.16) / 0.09, 0.0, 1.0)
    return float(1.4 * vx + tracking + 0.25 * stability - 0.65 * abs(vy)
                 - 0.5 * yaw_cost - 4.0 * env.fell)


def collect(env, policy, actor, critic, rng, cfg):
    batch = {key: [] for key in ('features', 'actions', 'advantages', 'returns')}
    metrics = []
    actor_w = actor.weight.detach().numpy()
    actor_b = actor.bias.detach().numpy()
    for _ in range(cfg['episodes_per_iteration']):
        history = env.reset(int(rng.integers(2**31)))
        features, actions, rewards = [], [], []
        while not env.fell and env.data.time < cfg['episode_sim_seconds']:
            feature = policy.features(history)
            mean = actor_w @ feature + actor_b
            action = rng.normal(mean, cfg['action_std']).astype(np.float32)
            history = env.step(action)
            features.append(feature)
            actions.append(action)
            rewards.append(reward(env))
        with torch.no_grad():
            inputs = torch.from_numpy(np.vstack([*features, policy.features(history)]))
            values = critic(inputs).squeeze(-1).numpy()
        if env.fell:
            values[-1] = 0.0
        advantages = np.empty(len(rewards), dtype=np.float32)
        running_advantage = 0.0
        for index in reversed(range(len(rewards))):
            td_error = rewards[index] + cfg['gamma'] * values[index + 1] - values[index]
            running_advantage = td_error + cfg['gamma'] * cfg['gae_lambda'] * running_advantage
            advantages[index] = running_advantage
        batch['features'].extend(features)
        batch['actions'].extend(actions)
        batch['advantages'].extend(advantages)
        batch['returns'].extend(advantages + values[:-1])
        metrics.append((float(np.mean(rewards)), env.distance, env.fell))
    return {key: torch.from_numpy(np.asarray(value)) for key, value in batch.items()}, metrics


def validation(env, policy, actor, cfg):
    metrics = []
    weight = actor.weight.detach().numpy()
    bias = actor.bias.detach().numpy()
    for episode in range(cfg['validation_episodes']):
        history = env.reset(cfg['validation_seed'] + episode)
        total_reward = 0.0
        while not env.fell and env.data.time < cfg['episode_sim_seconds']:
            history = env.step(weight @ policy.features(history) + bias)
            total_reward += reward(env)
        metrics.append((total_reward, env.distance, env.fell))
    return summarize(metrics)


def summarize(metrics):
    rewards, distances, falls = np.asarray(metrics).T
    return {'reward_mean': float(rewards.mean()), 'forward_m_mean': float(distances.mean()),
            'falls': int(falls.sum())}


def train(args, cfg):
    torch.set_num_threads(1)
    torch.manual_seed(cfg['seed'])
    rng = np.random.default_rng(cfg['seed'])
    policy = WalkingPolicy(args.baseline)
    actor = nn.Linear(128, 12)
    critic = nn.Sequential(nn.Linear(128, 64), nn.Tanh(), nn.Linear(64, 1))
    with torch.no_grad():
        actor.weight.copy_(torch.from_numpy(policy.body[-1][0]))
        actor.bias.copy_(torch.from_numpy(policy.body[-1][1]))
    actor_optimizer = torch.optim.Adam(actor.parameters(), lr=cfg['actor_lr'])
    critic_optimizer = torch.optim.Adam(critic.parameters(), lr=cfg['critic_lr'])
    env = LocalGo1(args.model_dir, cfg['speed_mps'])
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'config.json').write_text(json.dumps(cfg, indent=2) + '\n', encoding='utf-8')

    def write_event(event, logfile):
        line = json.dumps(event)
        logfile.write(line + '\n')
        logfile.flush()
        print(line, flush=True)

    def save(path):
        with np.load(args.baseline, allow_pickle=False) as baseline:
            weights = dict(baseline)
        weights['body_3_w'] = actor.weight.detach().numpy()
        weights['body_3_b'] = actor.bias.detach().numpy()
        np.savez_compressed(path, **weights)

    with (args.output / 'metrics.jsonl').open('w', encoding='utf-8') as logfile:
        initial = validation(env, policy, actor, cfg)
        write_event({'iteration': 0, 'validation': initial,
                     'baseline_sha256': hashlib.sha256(args.baseline.read_bytes()).hexdigest()}, logfile)
        best = (-initial['falls'], initial['reward_mean'])
        updates = 0
        for iteration in range(1, cfg['iterations'] + 1):
            batch, metrics = collect(env, policy, actor, critic, rng, cfg)
            features, actions = batch['features'], batch['actions']
            advantages = batch['advantages']
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-6)
            with torch.no_grad():
                old_logp = Independent(Normal(actor(features), cfg['action_std']), 1).log_prob(actions)
                reference = features @ torch.from_numpy(policy.body[-1][0]).T
                reference += torch.from_numpy(policy.body[-1][1])
            for _ in range(cfg['epochs']):
                for index in torch.randperm(len(features)).split(cfg['batch_size']):
                    predicted_mean = actor(features[index])
                    distribution = Independent(Normal(predicted_mean, cfg['action_std']), 1)
                    ratio = torch.exp(distribution.log_prob(actions[index]) - old_logp[index])
                    clipped_ratio = ratio.clamp(1.0 - cfg['clip_range'], 1.0 + cfg['clip_range'])
                    objective = torch.minimum(ratio * advantages[index], clipped_ratio * advantages[index])
                    actor_loss = -objective.mean() + cfg['anchor_weight'] * nn.functional.mse_loss(
                        predicted_mean, reference[index]
                    )
                    actor_optimizer.zero_grad()
                    actor_loss.backward()
                    nn.utils.clip_grad_norm_(actor.parameters(), 0.5)
                    actor_optimizer.step()
                    critic_loss = nn.functional.mse_loss(
                        critic(features[index]).squeeze(-1), batch['returns'][index]
                    )
                    critic_optimizer.zero_grad()
                    critic_loss.backward()
                    nn.utils.clip_grad_norm_(critic.parameters(), 1.0)
                    critic_optimizer.step()
                    updates += 1
            measured = validation(env, policy, actor, cfg)
            score = (-measured['falls'], measured['reward_mean'])
            improved = score > best
            save(args.output / f'iteration-{iteration:02d}.npz')
            if improved:
                best = score
                save(args.output / 'best.npz')
            write_event({'iteration': iteration, 'rollout_steps': len(features), 'updates': updates,
                         'rollout': summarize(metrics), 'validation': measured, 'best': improved,
                         'actor_weight_delta_l2': float(torch.linalg.vector_norm(
                             actor.weight.detach() - torch.from_numpy(policy.body[-1][0])
                         ))}, logfile)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('model_dir', type=Path)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('config', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    train(args, json.loads(args.config.read_text(encoding='utf-8')))


if __name__ == '__main__':
    main()
