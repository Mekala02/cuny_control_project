## Homework Assignment

With this homework assignment we aim to provide you with the experience in the development of basic geometric control algorithms; i.e. pure pursuit and stanley. For this purpose, the provided code in this repository already has the necessary ROS2 and Gazebo Sim communication codes. So, you could consider it an an example for your future work. Below are your specific tasks for this assignment.

### For Pure Pursuit Controller

**Task:** To develop a pure pursuit control algorithm by writing your code in the indicated section below (generate_control_output). This algorithm should calculate the steering angle required for the trajectory tracking.

**Instructions:**
- Utilize the waypoints stored in `self.waypoints` to determine the optimal steering angle for the vehicle to follow the predefined path.
- Ensure that you incorporate the `self.wheelbase` parameter in your code to accurately determine the vehicle's turning radius.

### Stanley Controller

**Task:** To develop a stanley control algorithm by writing your code in the indicated section below (generate_control_output). This algorithm should calculate the steering angle required for the trajectory tracking.

**Instructions:**
- Utilize the waypoints stored in `self.waypoints` to calculate the desired heading and cross-track error.

## Modifying Code Sections

### Pure Pursuit Controller (/src/controller/controller/pure_pursuit.py)
Student are expected to contribute to the following sections in the provided `pure_pursuit.py` file:

- Implementation of the PPC code `generate_control_output(self, current_x, current_y, yaw)` function.
- Tuning of following the parameters for the desired tracking performance:
    - `self.k` (proportional gain)
    - `self.linear_velocity` (constant linear velocity)
    - `self.lookahead_distance` (lookahead distance)

### Stanley Controller (/src/controller/controller/stanley.py)
Student are expected to contribute to the following sections in the provided `stanley.py` file:

- Implementation of the Stanley code`generate_control_output(self, current_x, current_y, yaw)` function.
- Tune following the parameters for the desired tracking performance:
    - `self.k` (proportional gain)
    - `self.linear_velocity` (constant linear velocity)
  
<b>Note: We recommend that you do not modify any other sections in given file. However, feel free to make improvements to the code (at your risk) by also indicating the modifications you made. Otherwise, to evaluate your algorithm we will use our own code in integration with your controller code. <b/>

## Testing and Evaluation

- Test your implementations in the provided simulated environment using ROS 2 and Gazebo.
- Investigate the effects of parameter tuning on the behavior of the vehicle.

By completing this assignment, you'll have reinforced your understanding of control strategies for autonomous vehicle navigation and gain practical experience in developing control algorithms for autonomous systems.

## Additional Notes:

- The provided code includes a pre-built racing track in the Gazebo simulation environment. Your task is to write Pure Pursuit and Stanley controllers to control the vehicle on this track.
- You are only required to write the Pure Pursuit and Stanley controllers. The remaining code is provided by the course staff.
- Explore the effects of parameter tuning (e.g., lookahead distance, k) to ensure optimal performance.
- Upon submission, your code will be tested by us. With the developed controllers, the vehicle is expected to complete the track under 8 minutes without deviating from the track.


## Dependencies

You need to install ROS 2 Humble and Gazebo Sim for this project.

- [ROS2 Humble installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Gazebo Sim installation instructions](https://gazebosim.org/docs/fortress/install_ubuntu)

ROS2 Package dependencies
```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-xacro ros-humble-joint-state-publisher* ros-humble-rqt*
```
## Clone the repo

```bash
git clone --recurse-submodules https://github.com/ITU-EMAV/control-project-<your_username>
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
### Launching the gazebo_launch.py script will open two windows: RViz and the Gazebo simulation environment (The initial loading time for the first run may take up to 10 minutes as it downloads the map.)
<img src="https://github.com/Mekala02/cuny_control_project/blob/master/docs/a.jpeg" title="Web_Interface" alt="Web_Interface"/>&nbsp;
### To position the camera near the vehicle, click the "Move To" button:
<img src="https://github.com/Mekala02/cuny_control_project/blob/master/docs/b.jpeg" title="Web_Interface" alt="Web_Interface"/>&nbsp;
### For illumination, use the provided button to light up the surroundings:
<img src="https://github.com/Mekala02/cuny_control_project/blob/master/docs/c.jpeg" title="Web_Interface" alt="Web_Interface"/>&nbsp;
<br/>
<img src="https://github.com/Mekala02/cuny_control_project/blob/master/docs/d.jpeg" title="Web_Interface" alt="Web_Interface"/>&nbsp;


- To execute the Pure Pursuit and Stanley controllers, you'll need to implement them first. Once implemented, use the following commands. Ensure that you've launched the gazebo_launch.py script beforehand. The gazebo.launch.py script should be running simultaneously with the Pure Pursuit or Stanley controller.
Also, remember to source the environment with ```source install/setup.bash``` before running the controllers.
- Pure Pursuit
```bash
ros2 run controller pure_pursuit
```
- Stanley:
```bash
ros2 run controller stanley
```
### After running the controller, you can observe the reference tracking errors on the x-axis, y-axis, and as Euclidean distance.
<img src="docs/rqt.png" title="rqt_gui" alt="rqt_gui"/>&nbsp;
<br/>

## How to submit?
- Simply push the changes you made to your cloned repository at: https://github.com/ITU-EMAV/control-project-<your_username>
