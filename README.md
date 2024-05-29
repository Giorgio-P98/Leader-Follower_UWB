# Leader-Follower_UWB
A Leader-follower strategy, using Ultra-wideband Qorvo Antenna Kit. The repository is divided in two ROS2 workspace, one for the [Gazebo Simulation](https://github.com/Giorgio-P98/Leader-Follower_UWB/tree/main/Leader-Follower_Simulation), and one for [real testing](https://github.com/Giorgio-P98/Leader-Follower_UWB/tree/main/Leader-Follower) on a quadrotor equiped with a Pixhawk Mini and a Raspberry companion board. Of course the drone needs to be equiped with the double UWB antenna of the kit correctly set, whereas the target is the other antenna. 

The other folders of the repository, contains the latex source of the [report](https://github.com/Giorgio-P98/Leader-Follower_UWB/tree/main/Report) related to this application, and the [extra codes](https://github.com/Giorgio-P98/Leader-Follower_UWB/blob/main/Plot%20code%20and%20data/README.md) devolped to extract the images contained in the report.

## Simulation
In order to run the simulation, as first, the [toolchain installation](https://github.com/lucasantoro/PX4-ROS2-Tutorial/blob/main/docs/toolchain_installation.md) needs to be performed. In adition, the following ROS2 packages needs to be installed
```
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro
```
After that, get this repository on your PC.
```
git clone https://github.com/Giorgio-P98/Leader-Follower_UWB.git --recursive
```
Then go inside the simulation folder and build the workspace
```
cd Leader-Follower_UWB/Leader-Follower_Simulation/
colcon build
```
To run the simulation, workspace source is needed
```
cd
source Leader-Follower_UWB/Leader-Follower_Simulation/install/setup.bash
```
run the simulation using the ROS2 launch file
```
ros2 launch offboard_control sim_with_drone.launch.py
```
The simulation will run, by default, with a flight height of 2.0 meters and a following distance of 1.5 meters. At the end, it automatically prompts a series of graphs of the simulation results, saving both figures and .csv data files in the `/UAV_Follower_Plot/` folder, automatically created in the user home directory by the code. 
