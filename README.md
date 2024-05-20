# Project Name

## Description

This project simulates and trains a robot in an Ignition Gazebo world using ROS (Robot Operating System). The robot, named "bouncy_bot", can be trained either manually or through reinforcement learning algorithms like Q-learning or Deep Q-learning (DQL). The simulation environment is launched using Ignition Gazebo, and the robot's movement can be controlled via ROS commands or AI scripts.

## Installation

1. Clone this repository to your local machine:

    ```bash
    git clone https://github.com/your_username/project_name.git
    ```

2. Navigate to the project directory:

    ```bash
    cd project_name
    ```

3. Install dependencies using pip:

    ```bash
    pip install -r requirements.txt
    ```

## Usage

First launch the world:

    ```bash
    cd worlds
    ign gazebo cp_world
    ```

1. Launch the simulation environment manually:

    ```bash
    ros2 launch bouncy_bot launches/spawn.launch.py
    ```

    OR

    The environment will be automatically launched when using the AI training scripts.

2. Manually control the robot's movement:

    ```bash
    python scripts/move_script.py
    ```

3. Train the robot using Q-learning:

    ```bash
    python scripts/q_learn_training.py
    ```

4. Train the robot using Deep Q-learning (DQL):

    ```bash
    python scripts/dq_learn_training.py
    ```

    ```

## Contributing

Contributions are welcome! I am new to ros2 and ignition and will either update this or make a new version when I get better. 
