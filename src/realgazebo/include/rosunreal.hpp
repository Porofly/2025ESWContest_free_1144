#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/monitoring.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <string>
#include <vector>
#include <cstdint>
#include <cmath>
#include <ignition/math.hh>

// UDP (Linux/POSIX)
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <unistd.h>   // close()
#include <cstring>    // std::memset, std::memcpy


struct MotorCfg {
  int index;
  double input_offset;
  double input_scaling;
  double zero_position_disarmed;
  double zero_position_armed;
};

// Sockets에 얹어 보낼 헤더(패딩 제거)
#pragma pack(push, 1)
struct RealGazeboPacketHeader {
  uint8_t vehicle_num;
  uint8_t vehicle_code;
  uint8_t data_type;   // 예: 1 = motors
};
#pragma pack(pop)

class RosUnreal : public rclcpp::Node {
public:
  RosUnreal();
  ~RosUnreal() override;

private:
  // --- params & config ---
  void loadYaml_();
  void SetupSendSocket_(int & sock, struct sockaddr_in & addr, int port);
  uint8_t getVehicleCode(const std::string & vehicle_type) const;
  bool sendToUE_(const void* data, size_t size);

  // --- ROS callbacks ---
  void onPwm_(const int drone_id, const std::string& drone_type, const px4_msgs::msg::ActuatorMotors::SharedPtr msg);
  void onPose_(const int drone_id, const std::string& drone_type, const px4_msgs::msg::Monitoring::SharedPtr msg);
  void onTarget_(const int target_id, const std::string& target_type, const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);
  void onDetect_(const int target_id, const std::string& target_type, const int detect_id, const std::string& detect_type, const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // --- TF ---
  ignition::math::Quaterniond QuaternionToUe(double roll, double pitch, double yaw);
  std::vector<double> LLH2NED(const std::vector<double>& LLH, const std::vector<double>& ref_LLH);
private:
  // params
  std::string package_name_;
  std::string model_yaml_;
  std::string scenario_yaml_;
  std::string px4_motor_output_;
  std::string drone_position_output_;
  std::string tag_position_output_;
  std::string detect_position_output_;
  double position_tolerance_;
  std::vector<double> ref_LLH = {0, 0, 0};

  // pose
  std::vector<rclcpp::Subscription<px4_msgs::msg::Monitoring>::SharedPtr> sub_pose_;
  std::vector<double> target_offset;

  // motor
  std::vector<rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr> sub_pwm_;
  unsigned int num_motor_joint_ = 0;

  // target
  std::vector<rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr> sub_target_;
  size_t moving_average_window_size_;
  std::deque<double> x_history_;
  std::deque<double> y_history_;
  double x_sum_;
  double y_sum_;
  double target_y_ue;
  double target_x_ue;

  // detect
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr> sub_detect_;
  bool target_flag = false;
  double detect_num = 0;

  // UE5 UDP params
  std::string unreal_ip_;
  int unreal_sock_;
  int unreal_port_;
  struct sockaddr_in unreal_addr_;

  // config
  std::vector<MotorCfg> motors_;

};
