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

#### License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (c) 2019-2020, Saif Sidhik

If you use this software, please cite it using [![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.3747459.svg)](https://doi.org/10.5281/zenodo.3747459).

   [fri-repo]: <https://github.com/justagist/franka_ros_interface>
   [fpd-repo]: <https://github.com/justagist/franka_panda_description>
   [libfranka-doc]: <https://frankaemika.github.io/docs/installation_linux.html#building-from-source>
   [franka-ros]: <https://frankaemika.github.io/docs/franka_ros.html
