#!/usr/bin/env python3
"""
MAPPO (Multi-Agent PPO) Training Script
Centralized Training, Decentralized Execution
"""

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions import Normal
from typing import List, Tuple, Dict
import argparse
import os
from collections import deque
import pickle


class ActorCriticNetwork(nn.Module):
    """Actor-Critic network for MAPPO"""

    def __init__(self, obs_dim: int, action_dim: int, hidden_dims: List[int] = [256, 256]):
        super().__init__()

        # Actor network
        actor_layers = []
        in_dim = obs_dim
        for hidden_dim in hidden_dims:
            actor_layers.extend([
                nn.Linear(in_dim, hidden_dim),
                nn.LayerNorm(hidden_dim),
                nn.ReLU()
            ])
            in_dim = hidden_dim

        self.actor_base = nn.Sequential(*actor_layers)
        self.actor_mean = nn.Linear(in_dim, action_dim)
        self.actor_logstd = nn.Parameter(torch.zeros(action_dim))

        # Critic network (takes global state)
        critic_layers = []
        in_dim = obs_dim  # For simplicity, use same obs
        for hidden_dim in hidden_dims:
            critic_layers.extend([
                nn.Linear(in_dim, hidden_dim),
                nn.LayerNorm(hidden_dim),
                nn.ReLU()
            ])
            in_dim = hidden_dim

        self.critic = nn.Sequential(*critic_layers, nn.Linear(in_dim, 1))

        # Initialize weights
        self._initialize_weights()

    def _initialize_weights(self):
        for m in self.modules():
            if isinstance(m, nn.Linear):
                nn.init.orthogonal_(m.weight, gain=np.sqrt(2))
                nn.init.constant_(m.bias, 0)

    def forward(self, obs: torch.Tensor):
        # Actor forward
        actor_features = self.actor_base(obs)
        action_mean = torch.tanh(self.actor_mean(actor_features))  # Bounded to [-1, 1]
        action_std = torch.exp(self.actor_logstd)

        # Critic forward
        value = self.critic(obs)

        return action_mean, action_std, value

    def get_action(self, obs: torch.Tensor, deterministic: bool = False):
        with torch.no_grad():
            action_mean, action_std, value = self.forward(obs)

            if deterministic:
                action = action_mean
            else:
                dist = Normal(action_mean, action_std)
                action = dist.sample()
                action = torch.clamp(action, -1.0, 1.0)

        return action.cpu().numpy(), value.cpu().numpy()

    def evaluate_actions(self, obs: torch.Tensor, actions: torch.Tensor):
        action_mean, action_std, value = self.forward(obs)

        dist = Normal(action_mean, action_std)
        log_prob = dist.log_prob(actions).sum(dim=-1, keepdim=True)
        entropy = dist.entropy().sum(dim=-1, keepdim=True)

        return value, log_prob, entropy


class MultiAgentSwarmEnv:
    """Multi-Agent Drone Swarm Environment"""

    def __init__(self, num_drones: int = 10, max_steps: int = 600):
        self.num_drones = num_drones
        self.max_steps = max_steps
        self.world_size = 50.0
        self.dt = 0.1

        # Dynamics
        self.max_velocity = 3.0
        self.max_acceleration = 2.0
        self.safety_distance = 2.0
        self.goal_tolerance = 0.5

        # State
        self.positions = None
        self.velocities = None
        self.goals = None
        self.current_step = 0

    def reset(self):
        # Random initial positions
        self.positions = np.random.uniform(
            -self.world_size, self.world_size, (self.num_drones, 3)
        )
        self.positions[:, 2] = np.random.uniform(0, 20, self.num_drones)

        # Random goals
        self.goals = np.random.uniform(
            -self.world_size, self.world_size, (self.num_drones, 3)
        )
        self.goals[:, 2] = np.random.uniform(0, 20, self.num_drones)

        # Ensure minimum distance
        for i in range(self.num_drones):
            while np.linalg.norm(self.goals[i] - self.positions[i]) < 10.0:
                self.goals[i] = np.random.uniform(-self.world_size, self.world_size, 3)
                self.goals[i, 2] = np.random.uniform(0, 20)

        self.velocities = np.zeros((self.num_drones, 3))
        self.current_step = 0

        return self._get_observations()

    def step(self, actions: np.ndarray):
        # actions: (num_drones, 3)
        assert actions.shape == (self.num_drones, 3)

        # Update all drones
        for i in range(self.num_drones):
            self._update_drone(i, actions[i] * self.max_velocity)

        self.current_step += 1

        # Get observations, rewards, dones
        obs = self._get_observations()
        rewards = self._compute_rewards()
        dones = self._check_dones()

        return obs, rewards, dones

    def _update_drone(self, drone_id: int, velocity_command: np.ndarray):
        # Clip velocity
        vel_norm = np.linalg.norm(velocity_command)
        if vel_norm > self.max_velocity:
            velocity_command = velocity_command / vel_norm * self.max_velocity

        # Acceleration
        acc = (velocity_command - self.velocities[drone_id]) / self.dt
        acc_norm = np.linalg.norm(acc)
        if acc_norm > self.max_acceleration:
            acc = acc / acc_norm * self.max_acceleration

        # Update
        self.velocities[drone_id] += acc * self.dt
        self.positions[drone_id] += self.velocities[drone_id] * self.dt

        # Bounds
        self.positions[drone_id] = np.clip(
            self.positions[drone_id],
            [-self.world_size, -self.world_size, 0],
            [self.world_size, self.world_size, 20]
        )

    def _get_observations(self):
        # Simple observation: ego state + nearest neighbor
        obs = []
        for i in range(self.num_drones):
            ego_obs = []

            # Ego state
            ego_obs.extend(self.positions[i] / self.world_size)
            ego_obs.extend(self.velocities[i] / self.max_velocity)
            ego_obs.extend((self.goals[i] - self.positions[i]) / (2 * self.world_size))

            dist_to_goal = np.linalg.norm(self.goals[i] - self.positions[i])
            ego_obs.append(dist_to_goal / (2 * self.world_size))

            # Nearest neighbor
            nearest_dist = float('inf')
            nearest_idx = None
            for j in range(self.num_drones):
                if i == j:
                    continue
                dist = np.linalg.norm(self.positions[j] - self.positions[i])
                if dist < nearest_dist:
                    nearest_dist = dist
                    nearest_idx = j

            if nearest_idx is not None:
                rel_pos = (self.positions[nearest_idx] - self.positions[i]) / (2 * self.world_size)
                rel_vel = (self.velocities[nearest_idx] - self.velocities[i]) / self.max_velocity
                ego_obs.extend(rel_pos)
                ego_obs.extend(rel_vel)
                ego_obs.append(nearest_dist / (2 * self.world_size))
            else:
                ego_obs.extend([0, 0, 0, 0, 0, 0, 1.0])

            obs.append(np.array(ego_obs, dtype=np.float32))

        return np.array(obs)

    def _compute_rewards(self):
        rewards = np.zeros(self.num_drones)

        for i in range(self.num_drones):
            # Distance to goal
            dist = np.linalg.norm(self.positions[i] - self.goals[i])
            rewards[i] -= dist * 0.1

            # Goal reached
            if dist < self.goal_tolerance:
                rewards[i] += 100.0

            # Collision penalty
            for j in range(self.num_drones):
                if i == j:
                    continue
                dist_ij = np.linalg.norm(self.positions[i] - self.positions[j])
                if dist_ij < self.safety_distance:
                    rewards[i] -= 100.0
                    break

        return rewards

    def _check_dones(self):
        # All drones done if timeout or all reached goal
        if self.current_step >= self.max_steps:
            return np.ones(self.num_drones, dtype=bool)

        # Check if all reached goal
        all_reached = True
        for i in range(self.num_drones):
            dist = np.linalg.norm(self.positions[i] - self.goals[i])
            if dist >= self.goal_tolerance:
                all_reached = False
                break

        if all_reached:
            return np.ones(self.num_drones, dtype=bool)

        return np.zeros(self.num_drones, dtype=bool)


class MAPPOTrainer:
    """MAPPO Trainer with Centralized Training"""

    def __init__(
        self,
        env: MultiAgentSwarmEnv,
        obs_dim: int,
        action_dim: int,
        num_agents: int,
        device: str = 'cuda'
    ):
        self.env = env
        self.num_agents = num_agents
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')

        # Create shared policy network
        self.policy = ActorCriticNetwork(obs_dim, action_dim).to(self.device)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=3e-4)

        # Hyperparameters
        self.gamma = 0.99
        self.gae_lambda = 0.95
        self.clip_ratio = 0.2
        self.vf_coef = 0.5
        self.ent_coef = 0.01
        self.max_grad_norm = 0.5

        # Buffers
        self.rollout_buffer_size = 2048
        self.reset_buffers()

    def reset_buffers(self):
        self.obs_buffer = []
        self.action_buffer = []
        self.reward_buffer = []
        self.value_buffer = []
        self.log_prob_buffer = []
        self.done_buffer = []

    def collect_rollouts(self, num_steps: int):
        self.reset_buffers()

        obs = self.env.reset()

        for _ in range(num_steps):
            obs_tensor = torch.FloatTensor(obs).to(self.device)

            # Get actions for all agents
            with torch.no_grad():
                actions, values = [], []
                for i in range(self.num_agents):
                    action, value = self.policy.get_action(obs_tensor[i:i+1])
                    actions.append(action[0])
                    values.append(value[0])

            actions = np.array(actions)
            values = np.array(values)

            # Step environment
            next_obs, rewards, dones = self.env.step(actions)

            # Store in buffer
            self.obs_buffer.append(obs)
            self.action_buffer.append(actions)
            self.reward_buffer.append(rewards)
            self.value_buffer.append(values)
            self.done_buffer.append(dones)

            obs = next_obs

            if dones.all():
                obs = self.env.reset()

    def compute_advantages(self):
        # Compute GAE advantages
        advantages = []
        returns = []

        for t in reversed(range(len(self.reward_buffer))):
            if t == len(self.reward_buffer) - 1:
                next_value = 0
            else:
                next_value = self.value_buffer[t + 1]

            delta = self.reward_buffer[t] + self.gamma * next_value * (1 - self.done_buffer[t]) - self.value_buffer[t]

            if t == len(self.reward_buffer) - 1:
                advantage = delta
            else:
                advantage = delta + self.gamma * self.gae_lambda * advantages[0]

            advantages.insert(0, advantage)
            returns.insert(0, advantage + self.value_buffer[t])

        return np.array(advantages), np.array(returns)

    def update(self, n_epochs: int = 10, batch_size: int = 64):
        # Compute advantages
        advantages, returns = self.compute_advantages()

        # Convert to tensors
        obs = torch.FloatTensor(np.array(self.obs_buffer)).to(self.device)
        actions = torch.FloatTensor(np.array(self.action_buffer)).to(self.device)
        old_values = torch.FloatTensor(np.array(self.value_buffer)).to(self.device)
        advantages = torch.FloatTensor(advantages).to(self.device)
        returns = torch.FloatTensor(returns).to(self.device)

        # Flatten for all agents
        obs = obs.reshape(-1, obs.shape[-1])
        actions = actions.reshape(-1, actions.shape[-1])
        old_values = old_values.reshape(-1, 1)
        advantages = advantages.reshape(-1, 1)
        returns = returns.reshape(-1, 1)

        # Normalize advantages
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

        # Update policy
        for _ in range(n_epochs):
            # Shuffle indices
            indices = torch.randperm(obs.shape[0])

            for start in range(0, obs.shape[0], batch_size):
                end = start + batch_size
                batch_indices = indices[start:end]

                batch_obs = obs[batch_indices]
                batch_actions = actions[batch_indices]
                batch_advantages = advantages[batch_indices]
                batch_returns = returns[batch_indices]

                # Evaluate actions
                values, log_probs, entropy = self.policy.evaluate_actions(batch_obs, batch_actions)

                # Policy loss (PPO clip)
                ratio = torch.exp(log_probs - log_probs.detach())
                policy_loss1 = -batch_advantages * ratio
                policy_loss2 = -batch_advantages * torch.clamp(
                    ratio, 1 - self.clip_ratio, 1 + self.clip_ratio
                )
                policy_loss = torch.max(policy_loss1, policy_loss2).mean()

                # Value loss
                value_loss = nn.MSELoss()(values, batch_returns)

                # Entropy loss
                entropy_loss = -entropy.mean()

                # Total loss
                loss = policy_loss + self.vf_coef * value_loss + self.ent_coef * entropy_loss

                # Update
                self.optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.policy.parameters(), self.max_grad_norm)
                self.optimizer.step()

        return {
            'policy_loss': policy_loss.item(),
            'value_loss': value_loss.item(),
            'entropy': -entropy_loss.item()
        }

    def save(self, filepath: str):
        torch.save({
            'policy_state_dict': self.policy.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
        }, filepath)

    def load(self, filepath: str):
        checkpoint = torch.load(filepath)
        self.policy.load_state_dict(checkpoint['policy_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_drones', type=int, default=10)
    parser.add_argument('--total_steps', type=int, default=10000000)
    parser.add_argument('--rollout_steps', type=int, default=2048)
    parser.add_argument('--save_freq', type=int, default=100000)
    parser.add_argument('--save_path', type=str, default='./models/mappo_swarm')
    parser.add_argument('--device', type=str, default='cuda')
    args = parser.parse_args()

    os.makedirs(args.save_path, exist_ok=True)

    print(f"Training MAPPO with {args.num_drones} drones")
    print(f"Device: {args.device}")

    # Create environment
    env = MultiAgentSwarmEnv(num_drones=args.num_drones)

    # Observation and action dimensions
    obs_dim = 17  # See _get_observations
    action_dim = 3

    # Create trainer
    trainer = MAPPOTrainer(
        env=env,
        obs_dim=obs_dim,
        action_dim=action_dim,
        num_agents=args.num_drones,
        device=args.device
    )

    # Training loop
    num_updates = args.total_steps // args.rollout_steps
    print(f"Total updates: {num_updates}")

    for update in range(num_updates):
        # Collect rollouts
        trainer.collect_rollouts(args.rollout_steps)

        # Update policy
        metrics = trainer.update()

        # Logging
        if update % 10 == 0:
            avg_reward = np.mean([sum(r) for r in trainer.reward_buffer])
            print(f"Update {update}/{num_updates} | Avg Reward: {avg_reward:.2f} | "
                  f"Policy Loss: {metrics['policy_loss']:.4f} | "
                  f"Value Loss: {metrics['value_loss']:.4f} | "
                  f"Entropy: {metrics['entropy']:.4f}")

        # Save checkpoint
        if (update + 1) % (args.save_freq // args.rollout_steps) == 0:
            save_path = os.path.join(args.save_path, f'mappo_{update+1}.pt')
            trainer.save(save_path)
            print(f"Saved checkpoint to {save_path}")

    # Save final model
    final_path = os.path.join(args.save_path, 'mappo_final.pt')
    trainer.save(final_path)
    print(f"Training complete! Final model saved to {final_path}")


if __name__ == "__main__":
    main()
