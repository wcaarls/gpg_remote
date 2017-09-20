#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// From https://github.com/DexterInd/GoPiGo3/blob/master/Software/Python/gopigo3.py
#define MOTOR_GEAR_RATIO           120
#define ENCODER_TICKS_PER_ROTATION   6
#define MOTOR_TICKS_PER_RADIAN     ((MOTOR_GEAR_RATIO*ENCODER_TICKS_PER_ROTATION)/(2*M_PI))

#define SERVO_CENTER_US            1500
#define SERVO_RANGE_US             1850
#define SERVO_US_PER_RADIAN        (SERVO_RANGE_US/M_PI)

struct GPGRemoteStatus
{
  uint32_t size;
  int32_t pos[2];
  uint32_t line[5];
}; 

struct GPGRemoteCommand
{
  uint32_t size;
  int32_t vel[2];
  uint32_t servo;
};

class GPGRemoteHW : public hardware_interface::RobotHW
{
  private:
    ros::NodeHandle nh_;
  
    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::VelocityJointInterface jnt_vel_interface_;
    hardware_interface::PositionJointInterface jnt_pos_interface_;
    
    double cmd_[3];
    double pos_[3];
    double vel_[3];
    double eff_[3];
    int line_[5];
    
    int conn_;
    double first_;
    
  public:
    GPGRemoteHW() : conn_(-1), first_(true) { }

    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
    virtual void read(const ros::Time & time, const ros::Duration &period);
    virtual void write(const ros::Time & time, const ros::Duration &period);
    
    std::vector<int> getLineSensor();

  private:
    int connect();
};
