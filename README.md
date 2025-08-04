# ihmc_hands_ros2
A ROS 2 package for controlling various robot hands. 
Includes ROS 2 interfaces for commands and statuses, 
helpful Java classes, meshes, and URDF descriptions of the hands.

## Supported Hands

- PSYONIC Ability Hands
- SAKE Robotics EZGrippers

## Using ihmc_hands_ros2 as a Git Submodule

You may add ihmc_hands_ros2 into your ROS 2 workspace as a git submodule as such:

```shell
# cd into the ROS 2 workspace source directory
cd <your_ros2_ws>/src

# Clone ihmc_hands_ros2 and set it up as a git submodule
git submodule add https://github.com/ihmcrobotics/ihmc_hands_ros2.git

# Ensure the submodule is initialized as a git repository locally
git submodule init

# The following is not necessary, but may be useful to set
git config --global submodule.recurse true
```
