#include "gpg_remote/State.h"

#include <controller_manager/controller_manager.h>

#include "gpg_remote/gpg_remote_hw.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpg_remote_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<gpg_remote::State>("state", 10);
  
  ROS_INFO("Initializing GPG3 remote node");

  GPGRemoteHW robot;
  if (!robot.init(nh, nh))
  {
    ROS_FATAL("GPG3 initialization failed");
    return 1;
  }
  
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(100);
  ros::Time ts = ros::Time::now();
  
  ROS_INFO("Spinning");
  while(ros::ok())
  {
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
  
    robot.read(ts, d);
    cm.update(ts, d);
    robot.write(ts, d);
    
    gpg_remote::State msg;
    msg.line = robot.getLineSensor();
    pub.publish(msg);
    
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
