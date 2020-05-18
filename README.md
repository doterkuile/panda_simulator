# Panda Simulator

## Deprecated Branch

Has outdated functionalities. May not be compatible with Franka ROS Interface.

Use only for ROS Kinetic install, and only if `kinetic-devel` branch fails.

### Dependencies

- *libfranka* (`apt install ros-${ROS_DISTRO}-libfranka` or [install from source][libfranka-doc])
- *franka-ros* (`apt install ros-${ROS_DISTRO}-franka-ros` or [install from source][libfranka-doc])

The following dependencies can be installed using the `.rosinstall` file (instructions in next section)

- [*franka_ros_interface*][fri-repo]
- [*franka_panda_description*][fpd-repo] (urdf and model files from *panda_description* package modified to work in Gazebo, and with the custom controllers)
- [*orocos-kinematics-dynamics*](https://github.com/orocos/orocos_kinematics_dynamics)

### Installation

1.Clone the repo:

```bash
    cd <catkin_ws>/src
    git clone https://github.com/justagist/panda_simulator
```

2.Update dependency packages:

```bash
    wstool init
    wstool merge panda_simulator/dependencies.rosinstall
    wstool up

    # use old ros-compatible version of kdl
    cd orocos_kinematics_dynamics && git checkout b35c424e77ebc5b7e6f1c5e5c34f8a4666fbf5bc
    cd ../.. && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

3.Once the dependencies are met, the package can be installed using catkin_make:

```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    catkin build # if catkin not found, install catkin tools (apt install python-catkin-tools)
    source devel/setup.bash
```
