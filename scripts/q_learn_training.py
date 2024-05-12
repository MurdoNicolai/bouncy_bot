#!/usr/bin/env python

import numpy as np
import random
import time
from launch_launchfile import launchfile
from terminal_cmd import delete_robot
from q_learn import QLearn
from Move_script import Move
from queue import Queue
import threading
import rclpy

def run_episode(q_agent, interact_with_env, render=False):
    time.sleep(1) #wait for fall
    interact_with_env.reset()

    state = interact_with_env.return_state()
    total_reward = 0
    not_done = 20

    while not_done:
        action = q_agent.chooseAction(state)
        interact_with_env.ai_input(action)
        time.sleep(0.2)
        next_state = interact_with_env.return_state()
        max_angular_velocity = 10
        max_movement = interact_with_env.max_movement_f() #highest rotational speed over time period
        reward = 1.0 - abs(max_movement) / max_angular_velocity
        print(f"Current Reward: {max_movement}")
        not_done -= 1
        q_agent.learn(state, action, reward, next_state)
        total_reward += reward
        state = next_state

        # if render:
        #     env.render()

    print("\n\nepisode finished\n\n")
    return (total_reward)

def main():
    # for i in range(3):
    #     launch_robot(run_episode, q_agent)
    #     delete_robot()

    #open_sim
    # initialize ROS2 communication
    rclpy.init()
    interact_with_env = Move()
    spin_thread = threading.Thread(target=rclpy.spin,args=(interact_with_env, ))
    spin_thread.daemon = True
    spin_thread.start()

    actions = ["z","s","qq","dd","a","e","b"]  # list of actions see movescript
    epsilon = 0.1  # Exploration constant
    alpha = 0.5    # Learning rate
    gamma = 0.9    # Discount factor
    q_agent = QLearn(actions, epsilon, alpha, gamma)
    num_episodes = 5

    launch_bouncy = launchfile(run_episode, (q_agent, interact_with_env))
    for episode in range(num_episodes):
        print("\n\nstart\n\n")

        launch_bouncy.launch_robot()
        print("\n\nend\n\n")
        total_reward = launch_bouncy.result
        print(f"Episode {episode + 1}/{num_episodes}, Total Reward: {total_reward}")
        # delete_robot()

    # Evaluation
    eval_episodes = 2
    total_rewards = []
    for _ in range(eval_episodes):
        launch_bouncy.launch_robot()
        total_reward = launch_bouncy.result
        total_rewards.append(total_reward)
    delete_robot()

    avg_reward = np.mean(total_rewards)
    print(f"Average Reward over {eval_episodes} evaluation episodes: {avg_reward}")

    spin_thread.join()
    interact_with_env.destroy_node()
    rclpy.shutdown()

    #close sim

if __name__ == "__main__":
    main()


