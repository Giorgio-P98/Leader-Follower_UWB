# Leader-Follower_UWB
A Leader-follower strategy, using Ultra-wideband Qorvo Antenna Kit. The repository is divided in two ROS2 workspace, one for the [Gazebo Simulation](https://github.com/Giorgio-P98/Leader-Follower_UWB/tree/main/Leader-Follower_Simulation), and one for [real testing](https://github.com/Giorgio-P98/Leader-Follower_UWB/tree/main/Leader-Follower) on a quadrotor equiped with a Pixhawk Mini and a Raspberry companion board. Of course the drone needs to be equiped with the double UWB antenna of the kit correctly set, whereas the target is the other antenna.

## Simulation
In order to run the simulation, as first, the [toolchain installation](https://github.com/lucasantoro/PX4-ROS2-Tutorial/blob/main/docs/toolchain_installation.md) needs to be performed. In adition the following ROS2 packages needs to be installed
```
sudo apt install ros2-humble-gazebo
```
After that simply get this repository on your PC.
```
git clone https://github.com/Giorgio-P98/Leader-Follower_UWB.git
```
Then go inside the simulation folder and build the workspace
```
cd Leader-Follower_UWB/Leader-Follower_Simulation/
colcon build
```
Then to run the simulation, workspace source is needed
```
cd
source Leader-Follower_UWB/Leader-Follower_Simulation/install/setup.bash
```
