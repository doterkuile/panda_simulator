#include "objectscanner.h"



int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_robot_node");
  ROS_INFO_STREAM("Node initialized");
  ros::NodeHandle nh;


  ObjectScanner object(nh);

  ros::AsyncSpinner spinner(0);
  spinner.start();

  while(ros::ok())
  {
      ros::spinOnce();

      if(object.receivedJointPositions())
      {
          object.sendToNeutral();
      }
  }
  ros::waitForShutdown();


  return 0;
}
