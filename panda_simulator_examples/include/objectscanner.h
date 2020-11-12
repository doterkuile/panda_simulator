#ifndef OBJECTSCANNER_H
#define OBJECTSCANNER_H
#include <ros/ros.h>
#include <iostream>
#include "sensor_msgs/JointState.h"
#include "franka_core_msgs/JointCommand.h"
#include "franka_core_msgs/RobotState.h"


class ObjectScanner
{
public:
    ObjectScanner(ros::NodeHandle &nh);
    void jointCallback(const sensor_msgs::JointState &inputMsg);
    void robotCallback(const franka_core_msgs::RobotState &inputMsg);
    void sendToNeutral();
    bool receivedJointPositions();
private:

    void configParams();
    ros::NodeHandle nh_;
    ros::Publisher pubJoint_;
    ros::Subscriber subJoint_;
    ros::Subscriber subRobot_;

    std::string jointSubTopic_;
    std::string robotSubTopic_;
    std::string jointPubTopic_;

    std::vector<std::string> jointNames_;
    std::vector<double> neutralPosition_;

    std::vector<double> jointVelocities_;
    std::vector<double> jointPositions_;
    bool receivedPositions_;



};

#endif // OBJECTSCANNER_H
