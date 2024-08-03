# Robot Control and Wall Following Project
## Introduction
This project is the assignment for session09 at inmind. It is based on Gazebo and serves the purpose of making the robot go to the nearest wall, and start moving along the walls, timing each lap.
## Nodes
- `wall_finder_server`: Receives request from `robot_driver` node to move the robot to the nearest wall.

- `lap_timer_server`: Starts the timer after 10 seconds from launching the package and sends feedback of elapsed time and result of total time once the lap is finished. Uses /odom to check if a lap is finished.

- `lap_timer_client`: Sends timer request and logs the timer in a text file in `src` every result. Repeats the timer every lap.

- `robot_driver`: Initially waits for the `wall_finder_server` to move the robot to the nearest wall then starts moving along the wall, correcting its position by turning left or right and moving forward to make sure it stays close to the wall.
 
## Features
- Timer for each lap and logging it in a text file.
- Correctly finds the closest wall and turns to it efficiently
- Robot adjusts position and stays oriented on the wall properly.

## Usage
- **Prerequisites:** Gazebo and turtlebot simulation
- **Set up turtle model:**
    ```bash
    export TURTLEBOT3_MODEL=burger
    ```
- **Run Turtle Bot simulation:**
    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1.launch.py
    ```
- **Clone the repository**:
    ```bash
    git clone https://github.com/RONYkGT/Rony-TurtleBot-CPP-Session09.git
    ```
    then
    ```bash
    cd Rony-TurtleBot-CPP-Session09/ros2_ws/src/
    ```
- **Set up source**:
    ```bash
    source /opt/ros/humble/setup.bash 
    source install/setup.bash
    ```
- **Build package**:
    ```bash
    colcon build
    ```
- **Launch the nodes**:
     ```bash
    ros2 launch session_09 robot_launch.py
    ```
