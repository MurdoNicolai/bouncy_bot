#!/usr/bin/env python

import numpy as np
import random
import time
from launch_launchfile import launchfile
from terminal_cmd import delete_robot
from dql_agent import DQLearn
from Move_script import Move
import threading
import rclpy

def run_episode(agent, interact_with_env, render=False):
    batch_size=4
    actions = ["z", "s", "qq", "dd", "a", "e", "b"]
    time.sleep(1) # Wait for fall
    interact_with_env.reset()

    state = interact_with_env.return_state()
    state = np.reshape(state, [1, agent.state_size])
    total_reward = 0
    not_done = 20
    print("\n\nepisode started\n\n")

    while not_done:
        action = agent.choose_action(state)
        interact_with_env.ai_input(actions[action])
        time.sleep(0.2)
        next_state = interact_with_env.return_state()
        next_state = np.reshape(next_state, [1, agent.state_size])
        reward = 1 - (abs(interact_with_env.robot_positionx + 12) +
                      abs(interact_with_env.robot_positiony - 8.5)) / 30
        done = not_done <= 1
        agent.learn(state, action, reward, next_state, done, batch_size)
        total_reward += reward
        state = next_state
        not_done -= 1
        print(f"Current Reward: {reward}")

    print("\n\nepisode finished\n\n")
    return total_reward

def main():
    # Initialize ROS2 communication
    rclpy.init()
    interact_with_env = Move()
    spin_thread = threading.Thread(target=rclpy.spin, args=(interact_with_env, ))
    spin_thread.daemon = True
    spin_thread.start()

    state_size = interact_with_env.state_size
    action_size = len(["z", "s", "qq", "dd", "a", "e", "b"])  # Number of possible actions
    epsilon = 1.0  # Exploration constant
    alpha = 0.5  # Learning rate
    gamma = 0.9    # Discount factor
    agent = DQLearn(state_size, action_size, epsilon, alpha, gamma)
    num_episodes = 10

    for episode in range(num_episodes):
        launch_bouncy = launchfile(run_episode, (agent, interact_with_env, False))
        print("\n\nstart\n\n")

        launch_bouncy.launch_robot()
        print("\n\nend\n\n")
        total_reward = launch_bouncy.result
        print(f"Episode {episode + 1}/{num_episodes}, Total Reward: {total_reward}")
        delete_robot()

        if episode % 10 == 0:
            agent.update_target_model()

    # Evaluation
    eval_episodes = 2
    total_rewards = []
    for _ in range(eval_episodes):
        launch_bouncy = launchfile(run_episode, (agent, interact_with_env, False))
        launch_bouncy.launch_robot()
        total_reward = launch_bouncy.result
        total_rewards.append(total_reward)
        delete_robot()

    avg_reward = np.mean(total_rewards)
    print(f"Average Reward over {eval_episodes} evaluation episodes: {avg_reward}")

    spin_thread.join()
    interact_with_env.destroy_node()
    rclpy.shutdown()
    print("done")

if __name__ == "__main__":
    main()
