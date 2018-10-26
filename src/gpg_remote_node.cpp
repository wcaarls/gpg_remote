#include "gpg_remote/State.h"
#include "sensor_msgs/LaserScan.h"

#include <controller_manager/controller_manager.h>

#include "gpg_remote/gpg_remote_hw.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpg_remote_node");
  ros::NodeHandle nh;
  
  ROS_INFO("Initializing GPG3 remote node");

  GPGRemoteHW robot;
  if (!robot.init(nh, nh))
  {
    ROS_FATAL("GPG3 initialization failed");
    return 1;
  }
  
  ros::Publisher state_pub = nh.advertise<gpg_remote::State>("state", 10);
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
  ros::Subscriber led_sub = nh.subscribe("led", 1000, &GPGRemoteHW::setLED, &robot);
  
  controller_manager::ControllerManager cm(&robot, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate rate(20);
  ros::Time ts = ros::Time::now();
  
  ROS_INFO("Spinning");
  while(ros::ok())
  {
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
  
    robot.read(ts, d);
    cm.update(ts, d);
    robot.write(ts, d);
    
    gpg_remote::State state;
    state.line = robot.getLineSensor();
    state.battery = robot.getBatteryVoltage();
    state.distance = robot.getDistanceSensor();
    state.light = robot.getLightSensor();
    state_pub.publish(state);
    
    sensor_msgs::LaserScan scan;
    scan.header.frame_id = "base_link";
    scan.header.stamp = ros::Time::now();
    scan.angle_min = 0;
    scan.angle_max = 1.5*M_PI;
    scan.angle_increment = 0.5*M_PI;
    scan.time_increment = 0.01;
    scan.scan_time = 0.01;
    scan.range_min = 0.25;
    scan.range_max = 2.;
    scan.ranges = state.distance;
    scan_pub.publish(scan);
    
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
