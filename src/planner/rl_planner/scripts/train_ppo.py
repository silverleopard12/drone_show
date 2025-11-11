#!/usr/bin/env python3
"""
PPO Training Script for Drone Swarm
Uses Stable-Baselines3 for Single-Agent baseline
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from typing import Tuple, Dict, Any
import argparse
import os


class DroneSwarmGymEnv(gym.Env):
    """
    Gym wrapper for drone swarm environment
    Single-agent version (controls one drone, others use simple policy)
    """

    metadata = {'render_modes': []}

    def __init__(
        self,
        num_drones: int = 10,
        num_neighbors: int = 5,
        max_steps: int = 600,
        world_size: float = 50.0,
        dt: float = 0.1,
        use_communication: bool = True
    ):
        super().__init__()

        self.num_drones = num_drones
        self.num_neighbors = num_neighbors
        self.max_steps = max_steps
        self.world_size = world_size
        self.dt = dt
        self.use_communication = use_communication

        # Dynamics
        self.max_velocity = 3.0  # m/s
        self.max_acceleration = 2.0  # m/s^2
        self.safety_distance = 2.0  # m
        self.goal_tolerance = 0.5  # m

        # State: [ego_pos(3), ego_vel(3), goal(3), dist_to_goal(1),
        #         neighbor_pos(3*k), neighbor_vel(3*k), neighbor_dist(k)]
        obs_dim = 10 + num_neighbors * 7
        if use_communication:
            obs_dim += num_neighbors * 3  # neighbor goals

        # Observation space: normalized to [-1, 1]
        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(obs_dim,),
            dtype=np.float32
        )

        # Action space: 3D velocity command
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(3,),
            dtype=np.float32
        )

        # State variables
        self.positions = None
        self.velocities = None
        self.goals = None
        self.reached_goal = None
        self.collided = None
        self.current_step = 0

        # Statistics
        self.episode_reward = 0.0
        self.total_collisions = 0

    def reset(
        self,
        seed: int = None,
        options: Dict[str, Any] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)

        # Initialize drone states
        self.positions = np.random.uniform(
            -self.world_size,
            self.world_size,
            (self.num_drones, 3)
        )
        self.positions[:, 2] = np.random.uniform(0, 20, self.num_drones)  # Height: 0-20m

        # Random goals (ensure minimum distance)
        self.goals = np.random.uniform(
            -self.world_size,
            self.world_size,
            (self.num_drones, 3)
        )
        self.goals[:, 2] = np.random.uniform(0, 20, self.num_drones)

        # Ensure goals are far enough
        for i in range(self.num_drones):
            while np.linalg.norm(self.goals[i] - self.positions[i]) < 10.0:
                self.goals[i] = np.random.uniform(-self.world_size, self.world_size, 3)
                self.goals[i, 2] = np.random.uniform(0, 20)

        self.velocities = np.zeros((self.num_drones, 3))
        self.reached_goal = np.zeros(self.num_drones, dtype=bool)
        self.collided = np.zeros(self.num_drones, dtype=bool)

        self.current_step = 0
        self.episode_reward = 0.0
        self.total_collisions = 0

        obs = self._get_observation(0)  # Control first drone
        info = self._get_info()

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        # Scale action from [-1, 1] to velocity range
        velocity_command = action * self.max_velocity

        # Apply action to ego drone (id=0)
        self._update_drone(0, velocity_command)

        # Simple policy for other drones: move towards goal
        for i in range(1, self.num_drones):
            if not self.reached_goal[i] and not self.collided[i]:
                direction = self.goals[i] - self.positions[i]
                distance = np.linalg.norm(direction)
                if distance > 0:
                    simple_action = (direction / distance) * self.max_velocity * 0.5
                    self._update_drone(i, simple_action)

        self.current_step += 1

        # Check collisions and goals
        for i in range(self.num_drones):
            if not self.collided[i]:
                if self._check_collision(i):
                    self.collided[i] = True
                    self.total_collisions += 1

            if not self.reached_goal[i]:
                if self._check_goal_reached(i):
                    self.reached_goal[i] = True

        # Compute reward for ego drone
        reward = self._compute_reward(0, action)
        self.episode_reward += reward

        # Check if episode is done
        terminated = bool(self.reached_goal[0] or self.collided[0])
        truncated = bool(self.current_step >= self.max_steps)

        obs = self._get_observation(0)
        info = self._get_info()

        return obs, reward, terminated, truncated, info

    def _update_drone(self, drone_id: int, velocity_command: np.ndarray):
        # Clip velocity command
        vel_norm = np.linalg.norm(velocity_command)
        if vel_norm > self.max_velocity:
            velocity_command = velocity_command / vel_norm * self.max_velocity

        # Compute acceleration
        acc = (velocity_command - self.velocities[drone_id]) / self.dt

        # Clip acceleration
        acc_norm = np.linalg.norm(acc)
        if acc_norm > self.max_acceleration:
            acc = acc / acc_norm * self.max_acceleration

        # Update velocity and position
        self.velocities[drone_id] += acc * self.dt
        self.positions[drone_id] += self.velocities[drone_id] * self.dt

        # Enforce bounds
        self.positions[drone_id] = np.clip(
            self.positions[drone_id],
            [-self.world_size, -self.world_size, 0],
            [self.world_size, self.world_size, 20]
        )

    def _check_collision(self, drone_id: int) -> bool:
        for i in range(self.num_drones):
            if i == drone_id:
                continue
            distance = np.linalg.norm(self.positions[drone_id] - self.positions[i])
            if distance < self.safety_distance:
                return True
        return False

    def _check_goal_reached(self, drone_id: int) -> bool:
        distance = np.linalg.norm(self.positions[drone_id] - self.goals[drone_id])
        return distance < self.goal_tolerance

    def _get_observation(self, drone_id: int) -> np.ndarray:
        ego_pos = self.positions[drone_id]
        ego_vel = self.velocities[drone_id]
        goal = self.goals[drone_id]

        # Ego state (normalized)
        obs = []
        obs.extend(ego_pos / self.world_size)  # 3
        obs.extend(ego_vel / self.max_velocity)  # 3
        obs.extend((goal - ego_pos) / (2 * self.world_size))  # 3 (relative goal)

        dist_to_goal = np.linalg.norm(goal - ego_pos)
        obs.append(dist_to_goal / (2 * self.world_size))  # 1

        # Get nearest neighbors
        distances = []
        for i in range(self.num_drones):
            if i == drone_id:
                continue
            dist = np.linalg.norm(self.positions[i] - ego_pos)
            distances.append((dist, i))

        distances.sort()
        neighbor_ids = [idx for _, idx in distances[:self.num_neighbors]]

        # Neighbor states
        for neighbor_id in neighbor_ids:
            rel_pos = (self.positions[neighbor_id] - ego_pos) / (2 * self.world_size)
            rel_vel = (self.velocities[neighbor_id] - ego_vel) / self.max_velocity
            distance = np.linalg.norm(self.positions[neighbor_id] - ego_pos)

            obs.extend(rel_pos)  # 3
            obs.extend(rel_vel)  # 3
            obs.append(distance / (2 * self.world_size))  # 1

            if self.use_communication:
                rel_goal = (self.goals[neighbor_id] - ego_pos) / (2 * self.world_size)
                obs.extend(rel_goal)  # 3

        # Pad if fewer neighbors
        while len(neighbor_ids) < self.num_neighbors:
            obs.extend([0, 0, 0])  # pos
            obs.extend([0, 0, 0])  # vel
            obs.append(1.0)  # large distance
            if self.use_communication:
                obs.extend([0, 0, 0])  # goal

        return np.array(obs, dtype=np.float32)

    def _compute_reward(self, drone_id: int, action: np.ndarray) -> float:
        reward = 0.0

        # Goal reached
        if self.reached_goal[drone_id]:
            return 100.0

        # Collision penalty
        if self.collided[drone_id]:
            return -100.0

        # Distance to goal (negative to encourage progress)
        distance_to_goal = np.linalg.norm(
            self.positions[drone_id] - self.goals[drone_id]
        )
        reward += -distance_to_goal * 0.1

        # Smoothness (penalize large actions)
        action_penalty = -np.sum(action ** 2) * 0.01
        reward += action_penalty

        # Collision avoidance (soft constraint)
        min_distance = float('inf')
        for i in range(self.num_drones):
            if i == drone_id:
                continue
            dist = np.linalg.norm(self.positions[drone_id] - self.positions[i])
            min_distance = min(min_distance, dist)

        if min_distance < self.safety_distance * 2:
            safety_reward = -(self.safety_distance * 2 - min_distance) * 0.5
            reward += safety_reward

        return reward

    def _get_info(self) -> Dict[str, Any]:
        return {
            'episode_reward': self.episode_reward,
            'total_collisions': self.total_collisions,
            'goal_reached': bool(self.reached_goal[0]),
            'collided': bool(self.collided[0]),
            'distance_to_goal': np.linalg.norm(
                self.positions[0] - self.goals[0]
            )
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--total_timesteps', type=int, default=1000000)
    parser.add_argument('--num_envs', type=int, default=8)
    parser.add_argument('--num_drones', type=int, default=10)
    parser.add_argument('--save_path', type=str, default='./models/ppo_swarm')
    parser.add_argument('--log_dir', type=str, default='./logs/ppo_swarm')
    parser.add_argument('--device', type=str, default='cuda' if torch.cuda.is_available() else 'cpu')
    args = parser.parse_args()

    # Create directories
    os.makedirs(args.save_path, exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)

    print(f"Training PPO on device: {args.device}")
    print(f"Number of parallel environments: {args.num_envs}")
    print(f"Number of drones: {args.num_drones}")

    # Create vectorized environment
    env = make_vec_env(
        lambda: DroneSwarmGymEnv(num_drones=args.num_drones),
        n_envs=args.num_envs,
        vec_env_cls=SubprocVecEnv if args.num_envs > 1 else DummyVecEnv
    )

    # Create PPO model
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        verbose=1,
        device=args.device,
        tensorboard_log=args.log_dir
    )

    # Callbacks
    checkpoint_callback = CheckpointCallback(
        save_freq=50000,
        save_path=args.save_path,
        name_prefix='ppo_swarm'
    )

    eval_env = make_vec_env(
        lambda: DroneSwarmGymEnv(num_drones=args.num_drones),
        n_envs=1
    )

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=args.save_path,
        log_path=args.log_dir,
        eval_freq=10000,
        n_eval_episodes=10,
        deterministic=True
    )

    # Train
    print("Starting training...")
    model.learn(
        total_timesteps=args.total_timesteps,
        callback=[checkpoint_callback, eval_callback],
        progress_bar=True
    )

    # Save final model
    final_path = os.path.join(args.save_path, 'ppo_swarm_final')
    model.save(final_path)
    print(f"Training complete! Model saved to {final_path}")

    # Test the model
    print("\nTesting trained model...")
    obs, _ = eval_env.reset()
    total_reward = 0
    for _ in range(600):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, truncated, info = eval_env.step(action)
        total_reward += reward[0]
        if done or truncated:
            break

    print(f"Test episode reward: {total_reward:.2f}")


if __name__ == "__main__":
    main()
