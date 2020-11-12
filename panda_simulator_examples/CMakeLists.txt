cmake_minimum_required(VERSION 2.8.3)
project(panda_simulator_examples)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    franka_core_msgs
    gazebo_msgs
    franka_moveit
    panda_sim_moveit
    panda_gazebo
)

include_directories(include ${catkin_INCLUDE_DIRS})


# Add executable
add_executable(move_arm src/move_arm.cpp src/objectscanner.cpp)

target_link_libraries(move_arm ${catkin_LIBRARIES})



install(PROGRAMS
   scripts/move_robot.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

