#!/usr/bin/env python

import time
import numpy
import random
import time
from launch_launchfile import launch_robot
from terminal_cmd import delete_robot
from q_learn import QLearn
from env import Environment


def learn(launchfile):
    print("\n\n\n start \n\n\n")
    time.sleep(3)
    print("\n\n\n done! \n\n\n")
    launchfile.shutdown()

def run_episode(env, q_agent, render=False):
    state = env.reset()
    total_reward = 0
    done = False

    while not done:
        action = q_agent.chooseAction(state)
        next_state, reward, done, _ = env.step(action)
        q_agent.learn(state, action, reward, next_state)
        total_reward += reward
        state = next_state

        if render:
            env.render()

    return total_reward

def main():
    for i in range(3):
        launch_robot(learn)
        delete_robot()


    last_time_steps = numpy.ndarray(0)

    actions = [0, 1, 2]  # Example list of actions
    epsilon = 0.1  # Exploration constant
    alpha = 0.5    # Learning rate
    gamma = 0.9    # Discount factor
    q_agent = QLearn(actions, epsilon, alpha, gamma)
    num_episodes = 5


    for episode in range(num_episodes):
        total_reward = run_episode(env, q_agent)
        print(f"Episode {episode + 1}/{num_episodes}, Total Reward: {total_reward}")

    # Evaluation
    eval_episodes = 10
    total_rewards = []
    for _ in range(eval_episodes):
        total_reward = run_episode(env, q_agent, render=True)
        total_rewards.append(total_reward)
    avg_reward = np.mean(total_rewards)
    print(f"Average Reward over {eval_episodes} evaluation episodes: {avg_reward}")

    env.close()

if __name__ == "__main__":
    main()
