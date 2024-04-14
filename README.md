# control_lecture

## Homework Assignment

For this homework assignment, you'll be implementing crucial functions within both the Pure Pursuit and Stanley Controllers. Below are the tasks for each controller:

### Pure Pursuit Controller

**Task:** Implement the `generate_control_output` function to calculate the steering angle required for path following.

**Instructions:**
- Utilize the waypoints stored in `self.waypoints` to determine the optimal steering angle for the vehicle to follow the predefined path.
- Ensure that you incorporate the `self.wheelbase` parameter in your calculations to accurately determine the vehicle's turning radius.

### Stanley Controller

**Task:** Implement the `generate_control_output` function to calculate the steering angle required for path following.

**Instructions:**
- Utilize the waypoints stored in `self.waypoints` to calculate the desired heading and cross-track error.
  
### Testing and Evaluation

- Test your implementations in a simulated environment using ROS 2 and Gazebo.
- Explore parameter tuning to observe the behavior of the vehicle under different scenarios.

By completing this assignment, you'll reinforce your understanding of control strategies for autonomous vehicle navigation and gain practical experience in developing control algorithms for autonomous systems.

## Dependencies

You need to install ROS 2 humble and Gazebo Sim for this project.

- [ROS2 Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo Sim installation instructions](https://gazebosim.org/docs/fortress/install_ubuntu)

ROS2 Package dependencies
```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-xacro ros-humble-joint-state-publisher* ros-humble-rqt*
```

## Building the packages

Build the package and source install/setup.bash.
```bash
colcon build --packages-select gazebo_project controller messages --symlink-install && source install/setup.bash
```

- To run the gazebo simulation and rviz:
```bash
ros2 launch launch/gazebo_launch.py

```

- Run pure pursuit:
```bash
ros2 run controller pure_pursuit
```
- Run stanley:
```bash
ros2 run controller stanley
```

