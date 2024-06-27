# Robot_Stage_Laser
## Description
A basic navigation algorithm is implemented in this repository. The simulated environment can be downloaded in a [tuw-robotics](https://github.com/tuw-robotics/stage_ros2) repository.

## Prerequisites
- ROS2 Humble Hawksbill.
- [stage_ros2](https://github.com/tuw-robotics/stage_ros2) package. 

## Installation Steps

1. Clone this repository:
    ```sh
    git clone https://github.com/Gardiy/Laser-Stage.git
    cd nombre_del_repositorio
    ```

2. Build the package:
    ```sh
    colcon build
    ```

3. Source the setup file:
    ```sh
    source install/setup.bash
    ```

4. Run the stage simulator with the following command:
    ```sh
    ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
    ```

## Usage

After setting up the environment, you can run the main node of this package using:
```sh
ros2 launch robot_stage robot.launch.py
  ```
## Video

Implementation of a basic navigation algorithm [(link)]https://github.com/tuw-robotics/stage_ros2) .
