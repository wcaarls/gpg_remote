#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <pluginlib/class_list_macros.h>

#include "gpg_remote/gpg_remote_hw.h"

int GPGRemoteHW::connect()
{
  int optval = 1;

  struct sockaddr_in addr;
  struct hostent *server;

  int fd = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval);

  bzero((char *) &addr, sizeof(addr));
  addr.sin_family = AF_INET;
  
  std::string host_name, port_number="8002";
  nh_.getParam("host", host_name);
  nh_.getParam("data_port", port_number);

  server = gethostbyname(host_name.c_str());
  if (!server)
  {
    ROS_ERROR_STREAM("Could not resolve GPG3 hostname '" << host_name << "'");
    return -1;
  }

  bcopy((char *)server->h_addr, (char *)&addr.sin_addr.s_addr, server->h_length);
  addr.sin_port = htons(atoi(port_number.c_str()));

  ROS_INFO_STREAM("Connecting to GPG3 at " << host_name << ":" << port_number);
  if (::connect(fd,(struct sockaddr *) &addr,sizeof(addr)) < 0)
  {
    ROS_ERROR_STREAM("Could not connect to GPG3: " << strerror(errno));
    return -1;
  }

  return fd;
}

bool GPGRemoteHW::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh)
{
  ROS_INFO("Initializing GPG3 hardware interface");
  
  nh_ = robot_hw_nh;

  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_left("left_wheel", &pos_[0], &vel_[0], &eff_[0]);
  jnt_state_interface_.registerHandle(state_handle_left);

  hardware_interface::JointStateHandle state_handle_right("right_wheel", &pos_[1], &vel_[1], &eff_[1]);
  jnt_state_interface_.registerHandle(state_handle_right);

  hardware_interface::JointStateHandle state_handle_servo("servo", &pos_[2], &vel_[2], &eff_[2]);
  jnt_state_interface_.registerHandle(state_handle_servo);

  registerInterface(&jnt_state_interface_);

  // connect and register the joint velocity interface
  hardware_interface::JointHandle vel_handle_left(jnt_state_interface_.getHandle("left_wheel"), &cmd_[0]);
  jnt_vel_interface_.registerHandle(vel_handle_left);

  hardware_interface::JointHandle vel_handle_right(jnt_state_interface_.getHandle("right_wheel"), &cmd_[1]);
  jnt_vel_interface_.registerHandle(vel_handle_right);

  registerInterface(&jnt_vel_interface_);

  hardware_interface::JointHandle pos_handle_servo(jnt_state_interface_.getHandle("servo"), &cmd_[2]);
  jnt_pos_interface_.registerHandle(pos_handle_servo);

  registerInterface(&jnt_pos_interface_);

  conn_ = connect();
  first_ = true;
  
  return (conn_ >= 0);
}

void GPGRemoteHW::read(const ros::Time &time, const ros::Duration &period)
{
  int available;
  GPGRemoteStatus msg;

  ioctl(conn_, FIONREAD, &available);
  while (available >= sizeof(GPGRemoteStatus))
  {
    if (::read(conn_, &msg, sizeof(GPGRemoteStatus)) < sizeof(GPGRemoteStatus))
    {
      ROS_FATAL_STREAM("Could not read GPG3 status message: " << strerror(errno));
      exit(1);
    }

    if (msg.size != sizeof(GPGRemoteStatus)-4)
    {
      ROS_FATAL_STREAM("Illegal GPG3 status message size " << msg.size);
      exit(1);
    }

    // Wheels
    for (int ii=0; ii<2; ++ii)
    {
      if (first_)
        pos_[ii] = msg.pos[ii]/MOTOR_TICKS_PER_RADIAN;
    
      double prevpos = pos_[ii];
      pos_[ii] = msg.pos[ii]/MOTOR_TICKS_PER_RADIAN;
      vel_[ii] = (pos_[ii]-prevpos)/period.toSec();
      eff_[ii] = 0;
    }
    
    // Servo
    pos_[2] = cmd_[2];
    vel_[2] = 0;
    eff_[2] = 0;
    
    // Line sensor
    for (int ii=0; ii<5; ++ii)
      if (msg.line[ii] < 1024)
        line_[ii] = msg.line[ii];
      
    // Battery voltage
    battery_ = msg.battery;

    first_ = false;

    ioctl(conn_, FIONREAD, &available);
  }
}

void GPGRemoteHW::write(const ros::Time &time, const ros::Duration &period)
{
  GPGRemoteCommand msg;

  msg.size = sizeof(GPGRemoteCommand)-4;

  msg.vel[0] = cmd_[0]*MOTOR_TICKS_PER_RADIAN;
  msg.vel[1] = cmd_[1]*MOTOR_TICKS_PER_RADIAN;
  msg.servo  = SERVO_CENTER_US + cmd_[2]*SERVO_US_PER_RADIAN;

  if (::write(conn_, &msg, sizeof(GPGRemoteCommand)) < sizeof(GPGRemoteCommand))
    ROS_ERROR_STREAM("Could not write GPG3 command message: " << strerror(errno));
}

std::vector<int> GPGRemoteHW::getLineSensor()
{
  std::vector<int> val(5);

  for (int ii=0; ii<5; ++ii)
    val[ii] = line_[ii];
    
  return val;
}

float GPGRemoteHW::getBatteryVoltage()
{
  return battery_;
}

PLUGINLIB_EXPORT_CLASS(GPGRemoteHW, hardware_interface::RobotHW)
