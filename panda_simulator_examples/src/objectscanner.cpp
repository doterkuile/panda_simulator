#include "objectscanner.h"

ObjectScanner::ObjectScanner(ros::NodeHandle &nh):
    nh_(nh),
    receivedPositions_(false)
{
    this->configParams();

    pubJoint_ = nh_.advertise<franka_core_msgs::JointCommand>(jointPubTopic_, 100);
    subJoint_ = nh_.subscribe(jointSubTopic_,10, &ObjectScanner::jointCallback, this);
    subRobot_ = nh_.subscribe(robotSubTopic_,10, &ObjectScanner::robotCallback, this);




}

void ObjectScanner::configParams()
{
    nh_.param("scanner_config/joint_names",jointNames_,std::vector<std::string>{});
    nh_.param("scanner_config/neutral_pose", neutralPosition_, std::vector<double>{0,0,0});
    nh_.param("scanner_config/joint_publisher_topic",jointPubTopic_, std::string("/panda_simulator/custom_franka_state_controller/joint_states"));
    nh_.param("scanner_config/robot_subscriber_topic", robotSubTopic_, std::string("/panda_simulator/custom_franka_state_controller/robot_state"));
    nh_.param("scanner_config/joint_subscriber_topic",jointSubTopic_, std::string("/panda_simulator/custom_franka_state_controller/joint_states"));


}

void ObjectScanner::jointCallback(const sensor_msgs::JointState &inputMsg)
{

    std::vector<double> tempVel, tempPos;
    for(std::vector<std::string>::const_iterator name = jointNames_.begin(); name != jointNames_.end(); name++)
    {
        std::vector<std::string>::const_iterator msg_it = std::find(inputMsg.name.begin(), inputMsg.name.end(), *name);
        int idx = msg_it - inputMsg.name.begin();
        if(msg_it != inputMsg.name.end())
        {
            tempVel.push_back(inputMsg.velocity[idx]);
            tempPos.push_back(inputMsg.position[idx]);

        }
    }
    jointVelocities_ = tempVel;
    jointPositions_ = tempPos;
    receivedPositions_ = true;

}

void ObjectScanner::robotCallback(const franka_core_msgs::RobotState &inputMsg)
{
//    ROS_INFO_STREAM("============= Current robot state: ============");
//    ROS_INFO_STREAM("Cartesian vel: \n" << *inputMsg.O_dP_EE);
//    ROS_INFO_STREAM("Gravity compensation torques: \n" << *inputMsg.gravity);
//    ROS_INFO_STREAM("Cartesian vel: \n" << *inputMsg.O_dP_EE.end());
//    ROS_INFO_STREAM("Cartesian vel: \n" << *inputMsg.O_dP_EE.end());

}

bool ObjectScanner::receivedJointPositions()
{
    return receivedPositions_;
}

void ObjectScanner::sendToNeutral()
{
   ros::Publisher tempPub = nh_.advertise<franka_core_msgs::JointCommand>(jointPubTopic_, 100);
   franka_core_msgs::JointCommand pubMsg;
   pubMsg.names = jointNames_;
   pubMsg.position = neutralPosition_;
   pubMsg.mode = franka_core_msgs::JointCommand::POSITION_MODE;
   std::vector<double> positionDiff = neutralPosition_;
   int ii = 0;
   double delta_d = 0;
   for(; ii < positionDiff.size(); ii++)
   {
       positionDiff[ii] -= jointPositions_[ii];
       delta_d += abs(positionDiff[ii]);
   }

   while(delta_d > 0.01)
   {
       delta_d = 0;
       tempPub.publish(pubMsg);

       for(int jj{0}; jj < positionDiff.size(); jj++)
       {
           positionDiff[jj] -= jointPositions_[jj];
           delta_d += abs(positionDiff[jj]);
       }

   }


}
