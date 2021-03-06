#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/ColorRGBA.h>

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
  float battery;
}; 

struct GPGRemoteStatusGrove
{
  uint32_t size;
  int32_t pos[2];
  uint32_t line[5];
  float battery;
  uint32_t distance[4];
  uint32_t light[2];
}; 

struct GPGRemoteCommand
{
  uint32_t size;
  int32_t vel[2];
  uint32_t servo;
  int8_t led[3];
} __attribute__((packed));

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
    double led_[3];
    int line_[5];
    float battery_;
    float distance_[4];
    float light_[2];
    
    int conn_;
    bool first_;
    
  public:
    GPGRemoteHW() : conn_(-1), first_(true) { }

    virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
    virtual void read(const ros::Time & time, const ros::Duration &period);
    virtual void write(const ros::Time & time, const ros::Duration &period);
    
    std::vector<int> getLineSensor();
    float getBatteryVoltage();
    std::vector<float> getDistanceSensor();
    std::vector<float> getLightSensor();
    
    void setLED(const std_msgs::ColorRGBA::ConstPtr& msg);

  private:
    int connect();
};
