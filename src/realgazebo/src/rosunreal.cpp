#include "rosunreal.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <stdexcept>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

RosUnreal::RosUnreal()
: rclcpp::Node("rosunreal")
{
  // ---- Declare params ----
  this->declare_parameter<std::string>("package_name", "realgazebo");

  this->declare_parameter<std::string>("model_yaml", "yaml/models/x500.yaml");
  this->declare_parameter<std::string>("scenario_yaml", "yaml/scenario/uwb_drone.yaml");

  this->declare_parameter<std::string>("px4_motor_output", "/manager/out/actuator_motors");
  this->declare_parameter<std::string>("drone_position_output", "/manager/out/monitoring");
  this->declare_parameter<std::string>("tag_position_output", "/jfi/in/target");
  this->declare_parameter<std::string>("detect_position_output", "/detections_posearray");

  this->declare_parameter<double>("position_tolerance", 3);

  // unreal ip, port
  this->declare_parameter<std::string>("unreal_ip", "10.255.70.248");
  this->declare_parameter<int>("unreal_port", 5005);

  // ---- Get params ----
  this->get_parameter("package_name", package_name_);

  this->get_parameter("model_yaml", model_yaml_);
  this->get_parameter("scenario_yaml", scenario_yaml_);

  this->get_parameter("px4_motor_output", px4_motor_output_);
  this->get_parameter("drone_position_output", drone_position_output_);
  this->get_parameter("tag_position_output", tag_position_output_);
  this->get_parameter("detect_position_output", detect_position_output_);

  this->get_parameter("position_tolerance", position_tolerance_);

  this->get_parameter("unreal_ip", unreal_ip_);
  this->get_parameter("unreal_port", unreal_port_);

  moving_average_window_size_ = 10;

  // --- Load Scenario YAML ---
  std::string pkg_share = ament_index_cpp::get_package_share_directory(package_name_);
  std::filesystem::path yaml_path = std::filesystem::path(pkg_share) / scenario_yaml_;
  YAML::Node scenario_ = YAML::LoadFile(yaml_path.string());
  RCLCPP_INFO(get_logger(), "Loading YAML: %s", yaml_path.string().c_str());

  auto drone_list_ = scenario_["vehicles"];
  auto target_list_ = scenario_["targets"];
  auto detect_list_ = scenario_["detect"];
  sub_pose_.reserve(drone_list_.size());
  sub_target_.reserve(target_list_.size());
  sub_detect_.reserve(detect_list_.size());

  // ---- Load Drone YAML ----
  loadYaml_();

  // ---- Setup UDP socket ----
  SetupSendSocket_(unreal_sock_, unreal_addr_, unreal_port_);

  // ---- ROS I/O ----
  for(auto it = drone_list_.begin(); it != drone_list_.end(); ++it) {
    int drone_id_ = it->first.as<int>();
    std::string drone_type_ = it->second["type"].as<std::string>();
    std::string topic_prefix = "/drone" + std::to_string(drone_id_+1);

    // PWM 구독
    sub_pwm_.push_back(
      this->create_subscription<px4_msgs::msg::ActuatorMotors>(
        topic_prefix+px4_motor_output_, rclcpp::SensorDataQoS(),
        [this, drone_id_, drone_type_](px4_msgs::msg::ActuatorMotors::SharedPtr msg){
          this->onPwm_(drone_id_, drone_type_, msg);
        })
    );
    // POSE 구독
    sub_pose_.push_back(
      this->create_subscription<px4_msgs::msg::Monitoring>(
        topic_prefix+drone_position_output_, rclcpp::SensorDataQoS(),
        [this, drone_id_, drone_type_](px4_msgs::msg::Monitoring::SharedPtr msg){
          this->onPose_(drone_id_, drone_type_, msg);
        })
    );
  }
  int target_id_;
  std::string target_type_;
  for(auto it = target_list_.begin(); it != target_list_.end(); ++it) {
    target_id_ = it->first.as<int>();
    target_type_ = it->second["type"].as<std::string>();

    // Target 구독
    // sub_target_.push_back(
    //   this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    //     "/drone1"+tag_position_output_, rclcpp::SensorDataQoS(),
    //     std::bind(&RosUnreal::onTarget_, this, std::placeholders::_1)));
      sub_target_.push_back(
      this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
        "/drone1"+tag_position_output_, rclcpp::SensorDataQoS(),
        [this, target_id_, target_type_](px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
          this->onTarget_(target_id_, target_type_, msg);
        })
    );

  }

  for(auto it = detect_list_.begin(); it != detect_list_.end(); ++it) {
    int detect_id_ = it->first.as<int>();
    std::string detect_type_ = it->second["type"].as<std::string>();

    sub_detect_.push_back(
      this->create_subscription<geometry_msgs::msg::PoseArray>(
        detect_position_output_, rclcpp::SensorDataQoS(),
        [this, target_id_, target_type_, detect_id_, detect_type_](geometry_msgs::msg::PoseArray::SharedPtr msg) {
          this->onDetect_(target_id_, target_type_, detect_id_, detect_type_, msg);
        })
    );
  }
  RCLCPP_INFO(this->get_logger(), "RosUnreal ready. Sub: %s  UE: %s:%d  motors=%u",
              px4_motor_output_.c_str(), unreal_ip_.c_str(), unreal_port_, num_motor_joint_);
}

RosUnreal::~RosUnreal() {
  if (unreal_sock_ >= 0) { close(unreal_sock_); }
}

void RosUnreal::loadYaml_()
{
  std::string pkg_share = ament_index_cpp::get_package_share_directory(package_name_);
  std::filesystem::path yaml_path = std::filesystem::path(pkg_share) / model_yaml_;
  RCLCPP_INFO(get_logger(), "Loading YAML: %s", yaml_path.string().c_str());

  YAML::Node motor_list_ = YAML::LoadFile(yaml_path.string());
  if (!motor_list_ || !motor_list_["motors"] || !motor_list_["motors"].IsSequence()) {
    throw std::runtime_error("YAML error: 'motors' sequence not found");
  }

  motors_.clear();
  motors_.reserve(motor_list_["motors"].size());

  for (const auto& motor_ : motor_list_["motors"]) {
    MotorCfg cfg;
    cfg.index               = motor_["index"].as<int>();
    cfg.input_offset        = motor_["input_offset"].as<double>();
    cfg.input_scaling       = motor_["input_scaling"].as<double>();
    cfg.zero_position_disarmed = motor_["zero_position_disarmed"].as<double>();
    cfg.zero_position_armed = motor_["zero_position_armed"].as<double>();
    motors_.push_back(cfg);
  }
  num_motor_joint_ = motors_.size();

  RCLCPP_INFO(get_logger(), "Loaded %u motor entries.", num_motor_joint_);
}

void RosUnreal::onPose_(const int drone_id, const std::string& drone_type, const px4_msgs::msg::Monitoring::SharedPtr msg) {
  if (!msg) {
      RCLCPP_ERROR(get_logger(), "onPose_ callback for drone %d received a null message pointer!", drone_id);
      return; // msg가 비어있으면 즉시 함수 종료
  }
  //Header 설정
  const size_t pose_payload_size = sizeof(RealGazeboPacketHeader) + 7 * sizeof(float);
  std::vector<uint8_t> pose_buf(pose_payload_size);
  
  RealGazeboPacketHeader* pose_header = reinterpret_cast<RealGazeboPacketHeader*>(pose_buf.data());
  pose_header->vehicle_num = static_cast<uint8_t>(drone_id);
  pose_header->vehicle_code = getVehicleCode(drone_type);
  pose_header->data_type = 1; // pose 데이터 번호

  // Quaternion 설정 (PX4-UE5)
  double roll = msg->roll;
  double pitch = msg->pitch;
  double yaw = msg->head;

  ignition::math::Quaterniond q = QuaternionToUe(roll, pitch, yaw);

  // 위치 설정
  double x_ue = (msg->rtk_e);
  double y_ue = (msg->rtk_n);
  double z_ue = -(msg->rtk_d);
  if(drone_id == 0) {
    ref_LLH = {msg->ref_lat, msg->ref_lon, msg->ref_alt};
    target_offset = {(x_ue - msg->pos_y), (y_ue - msg->pos_x), (z_ue + msg->pos_z)};    
  }
  // std::vector<double> LLH = {msg->ref_lat, msg->ref_lon, msg->ref_alt};
  // std::vector<double> spawn_point_ = LLH2NED(LLH, ref_LLH);
  // double x_ue = (spawn_point_[1] + msg->pos_y - 50);
  // double y_ue = -(spawn_point_[0] + msg->pos_x + 50);
  // double z_ue = -(spawn_point_[2] + msg->pos_z);
  float pose_values[7] = {
    static_cast<float>(x_ue), static_cast<float>(y_ue), static_cast<float>(z_ue),
    static_cast<float>(q.X()), static_cast<float>(q.Y()), static_cast<float>(q.Z()), static_cast<float>(q.W())
  };
  std::memcpy(pose_buf.data() + sizeof(RealGazeboPacketHeader), pose_values, sizeof(pose_values));

  // // Pose 값 로깅
  // const float* pose_values_ptr = reinterpret_cast<const float*>(
  //     static_cast<const uint8_t*>(pose_buf.data()) + sizeof(RealGazeboPacketHeader)
  // );

  // RCLCPP_INFO(get_logger(),
  //             "pose_value : Position(x=%.2f, y=%.2f, z=%.2f) "
  //             "Quaternion(x=%.4f, y=%.4f, z=%.4f, w=%.4f)",
  //             pose_values_ptr[0], pose_values_ptr[1], pose_values_ptr[2],
  //             pose_values_ptr[3], pose_values_ptr[4], pose_values_ptr[5], pose_values_ptr[6]);

  sendToUE_(pose_buf.data(), pose_payload_size);
}

void RosUnreal::onTarget_(const int target_id, const std::string& target_type, const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
  const size_t target_payload_size = sizeof(RealGazeboPacketHeader) + 7 * sizeof(float);
  std::vector<uint8_t> target_buf(target_payload_size);

  RealGazeboPacketHeader* target_header = reinterpret_cast<RealGazeboPacketHeader*>(target_buf.data());
  target_header->vehicle_num = target_id;
  target_header->vehicle_code = getVehicleCode(target_type);
  target_header->data_type = 1;

  double lat = msg->position[0];
  double lon = msg->position[1];
  double alt = msg->position[2];
  std::vector<double> LLH = {lat, lon, alt};

  if(!ref_LLH.empty() && !LLH.empty()) {
    std::vector<double> target_point_ = LLH2NED(LLH, ref_LLH);
    // // 필터링 x
    // target_x_ue = target_point_[1] + target_offset[0];
    // target_y_ue = target_point_[0] + target_offset[1];
    double target_z_ue = target_point_[2] + target_offset[2];

    double new_x = target_point_[1] + target_offset[0];
    double new_y = target_point_[0] + target_offset[1];

    // 이동 평균
    x_history_.push_back(new_x);
    x_sum_ += new_x;
    
    y_history_.push_back(new_y);
    y_sum_ += new_y;

    if (x_history_.size() > moving_average_window_size_) {
      x_sum_ -= x_history_.front();
      x_history_.pop_front();
    }
    if (y_history_.size() > moving_average_window_size_) {
      y_sum_ -= y_history_.front();
      y_history_.pop_front();
    }

    if (!x_history_.empty()) {
      target_x_ue = x_sum_ / x_history_.size();
      target_y_ue = y_sum_ / y_history_.size();
    }
    // float target_values[7] = {
    //   static_cast<float>(target_x_ue), static_cast<float>(target_y_ue), static_cast<float>(target_z_ue),
    //   0, 0, 0, 1};
    // std::memcpy(target_buf.data() + sizeof(RealGazeboPacketHeader), target_values, sizeof(target_values));

    // const float* target_values_ptr = reinterpret_cast<const float*>(
    //       static_cast<const uint8_t*>(target_buf.data()) + sizeof(RealGazeboPacketHeader)
    // );
    // RCLCPP_INFO(get_logger(), "target_value : Position(x=%.2f, y=%.2f)",
    //             target_x_ue, target_y_ue);

    // if(sendToUE_(target_buf.data(), target_payload_size)) {
    //   RCLCPP_INFO(get_logger(),"Target sendto() success");
    // }
  }
}

void RosUnreal::onDetect_(const int target_id, const std::string& target_type, 
  const int detect_id, const std::string& detect_type, const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
  double x_offset_ref;
  double y_offset_ref;
  //detect
  const size_t detect_payload_size = sizeof(RealGazeboPacketHeader) + 7 * sizeof(float);
  std::vector<uint8_t> detect_buf(detect_payload_size);
  std::vector<geometry_msgs::msg::Pose> pose_sort;

  //target detect
  const size_t target_payload_size = sizeof(RealGazeboPacketHeader) + 7 * sizeof(float);
  std::vector<uint8_t> target_buf(target_payload_size);

  // YOLO Box 정렬
  for (int i = 0; i < msg->poses.size(); ++i) {
    const geometry_msgs::msg::Pose& pose = msg->poses[i];
    pose_sort.push_back(pose);
  }
  std::sort(pose_sort.begin(), pose_sort.end(), [](const auto& a, const auto& b) {
    return std::tie(a.position.y, a.position.x) < std::tie(b.position.y, b.position.x);
  });

  // Target, 검출 좌표 거리 측정
  for(int i = 0; i < pose_sort.size(); i++) {
    if(i == 0)
      target_flag = true;
    RealGazeboPacketHeader* detect_header = reinterpret_cast<RealGazeboPacketHeader*>(detect_buf.data());
    detect_header->vehicle_num = detect_id + i;
    detect_header->vehicle_code = getVehicleCode(detect_type);
    detect_header->data_type = 1;

    double x_ue = pose_sort[i].position.y + target_offset[0];
    double y_ue = pose_sort[i].position.x + target_offset[1];
    double z_ue = pose_sort[i].position.z;

    double dx = x_ue - target_x_ue;
    double dy = y_ue - target_y_ue;
    double dist = dx*dx + dy*dy;

    // RCLCPP_INFO(this->get_logger(),
    //             "[%zu] Detect position: x=%.3f, y=%.3f, z=%.3f",
    //             i, x_ue, y_ue, z_ue);

    // 타겟 객체 스폰
    if(dist <= position_tolerance_*position_tolerance_ && target_flag) {
      x_offset_ref = x_ue;
      y_offset_ref = y_ue;

      RCLCPP_INFO(this->get_logger(), "Position Defference Distance: %.3f", dist);
      RCLCPP_INFO(this->get_logger(), "Target Detect Success");
      RealGazeboPacketHeader* target_header = reinterpret_cast<RealGazeboPacketHeader*>(target_buf.data());
      target_header->vehicle_num = target_id;
      target_header->vehicle_code = getVehicleCode(target_type);
      target_header->data_type = 1;

      float target_values[7] = {
      static_cast<float>(target_x_ue), static_cast<float>(target_y_ue), static_cast<float>(z_ue),
      0, 0, 0, 1};
      std::memcpy(target_buf.data() + sizeof(RealGazeboPacketHeader), target_values, sizeof(target_values));
      if(sendToUE_(target_buf.data(), target_payload_size)) {
        RCLCPP_INFO(get_logger(),"Target sendto() success");
      }

      // 기존 스폰 객체 -15m 이동
      float pose_values[7] = {
        static_cast<float>(target_x_ue), static_cast<float>(target_y_ue), static_cast<float>(-15),
        0, 0, 0, 1};
      std::memcpy(detect_buf.data() + sizeof(RealGazeboPacketHeader), pose_values, sizeof(pose_values));
      sendToUE_(detect_buf.data(), detect_payload_size);
      target_flag = false;
    }
  }
  // 감지 객체 전체 스폰
  for(int i = 0; i < pose_sort.size(); i++) {
    RealGazeboPacketHeader* detect_header = reinterpret_cast<RealGazeboPacketHeader*>(detect_buf.data());
    detect_header->vehicle_num = detect_id + i;
    detect_header->vehicle_code = getVehicleCode(detect_type);
    detect_header->data_type = 1;

    double x_ue = pose_sort[i].position.y + target_offset[0];
    double y_ue = pose_sort[i].position.x + target_offset[1];
    double z_ue = pose_sort[i].position.z;

    if(target_flag) {
        float pose_values[7] = {
        static_cast<float>(x_ue), static_cast<float>(y_ue), static_cast<float>(z_ue),
        0, 0, 0, 1};

        std::memcpy(detect_buf.data() + sizeof(RealGazeboPacketHeader), pose_values, sizeof(pose_values));
        if(sendToUE_(detect_buf.data(), detect_payload_size)) {
          RCLCPP_INFO(get_logger(),"Detect [%d] sendto() success", i);
        }
    }
    else {
      if(x_offset_ref != x_ue && y_offset_ref != y_ue) {
        double x_offset = x_ue-x_offset_ref;
        double y_offset = y_ue-y_offset_ref;

        float pose_values[7] = {
        static_cast<float>(target_x_ue+x_offset), static_cast<float>(target_y_ue+y_offset), static_cast<float>(z_ue),
        0, 0, 0, 1};

        std::memcpy(detect_buf.data() + sizeof(RealGazeboPacketHeader), pose_values, sizeof(pose_values));
        sendToUE_(detect_buf.data(), detect_payload_size);
      }
    }
  }
}

void RosUnreal::onPwm_(const int drone_id, const std::string& drone_type, const px4_msgs::msg::ActuatorMotors::SharedPtr msg)
{
  // 헤더
  const size_t motor_payload_size = sizeof(RealGazeboPacketHeader) + num_motor_joint_ * sizeof(float);
  std::vector<uint8_t> motor_buf(motor_payload_size);

  RealGazeboPacketHeader* motor_header = reinterpret_cast<RealGazeboPacketHeader*>(motor_buf.data());
  motor_header->vehicle_num = static_cast<uint8_t>(drone_id);
  motor_header->vehicle_code = getVehicleCode(drone_type);
  motor_header->data_type = 2; // motor 데이터 번호

  float* motor_data_ptr = reinterpret_cast<float*>(motor_buf.data() + sizeof(RealGazeboPacketHeader));

  for (size_t k = 0; k < num_motor_joint_; ++k) {
    const auto& motor_joint_list = motors_[k];

    const double u01 = msg->control[motor_joint_list.index];
    // --- [로깅 추가] ---
    // RCLCPP_INFO(this->get_logger(), "Motor[%zu] (index: %d) | Raw input u01: %.4f",
    //             k, motor_joint_list.index, u01);
    // -------------------
    // ((output-1000)/(2000-1000) - input_offset) * input_scaling + zero_position_armed
    double motor_velocity_ = 
        ((u01 - motor_joint_list.input_offset) * motor_joint_list.input_scaling + motor_joint_list.zero_position_armed) / 10;
    if(u01 <= 0.002)
      motor_velocity_ = motor_joint_list.zero_position_disarmed;

    motor_data_ptr[k] = static_cast<float>(motor_velocity_);  
  }
  // --- 로그 출력 ---
  // std::ostringstream oss;
  // oss << "Motor velocities [drone_id=" << drone_id << "]: ";
  // for (size_t k = 0; k < num_motor_joint_; ++k) {
  //   oss << motor_data_ptr[k];
  //   if (k != num_motor_joint_ - 1) oss << ", ";
  // }
  // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());
  // ------------------
  if(sendToUE_(motor_buf.data(), motor_payload_size)) {
      // RCLCPP_INFO(get_logger(),"PWM sendto() success");
  }
}

bool RosUnreal::sendToUE_(const void* data, size_t size)
{
  if (unreal_sock_ < 0) return false;
  ssize_t n = sendto(unreal_sock_, data, size, 0,
                     reinterpret_cast<struct sockaddr*>(&unreal_addr_),
                     sizeof(unreal_addr_));
  if (n < 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                         "sendto() failed");
    return false;
  }
  else {
    // RCLCPP_INFO(get_logger(),"sendto() success");
    return true;
  }
}

void RosUnreal::SetupSendSocket_(int & sock, struct sockaddr_in & addr, int port) {
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    RCLCPP_ERROR(get_logger(), "Failed to create UDP socket");
    return;
  }
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (inet_pton(AF_INET, unreal_ip_.c_str(), &addr.sin_addr) != 1) {
    RCLCPP_ERROR(get_logger(), "inet_pton failed for IP: %s", unreal_ip_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "UDP to UE5: %s:%d", unreal_ip_.c_str(), port);
  }
}

uint8_t RosUnreal::getVehicleCode(const std::string & vehicle_type) const
{
  if (vehicle_type == "iris") return 0;
  else if (vehicle_type == "x500") return 1;
  else if (vehicle_type == "rover") return 2;
  else if (vehicle_type == "boat") return 3;
  else if (vehicle_type == "lc_62") return 4;
  else if (vehicle_type == "wamv") return 5;
  else if (vehicle_type == "ugv_kimm") return 6;
  else if (vehicle_type == "person") return 7;
  else if (vehicle_type == "tag") return 8;
  else return 255;
}

ignition::math::Quaterniond RosUnreal::QuaternionToUe(double roll, double pitch, double yaw) {
    ignition::math::Quaterniond q_px4(ignition::math::Vector3d(roll, pitch, yaw));
    
    ignition::math::Matrix3d R_px4(q_px4);

    ignition::math::Matrix3d M(
        0,  1,  0,
       -1,  0,  0,
        0,  0, -1);
    
    ignition::math::Matrix3d R_ue = M * R_px4 * M.Transposed();
    
    return ignition::math::Quaterniond(R_ue);
}

std::vector<double> RosUnreal::LLH2NED(const std::vector<double>& LLH, const std::vector<double>& ref_LLH) {
  double a = 6378137.0;
  double f = 1.0 / 298.257223563;
  double e2 = 2 * f - f * f;

  // PI 상수 정의
  const double PI = 3.14159265358979323846;

  // --- Degree를 Radian으로 변환하는 단계 ---
  double lat_ref = ref_LLH[0] * PI / 180.0;
  double lon_ref = ref_LLH[1] * PI / 180.0;
  double lat = LLH[0] * PI / 180.0;
  double lon = LLH[1] * PI / 180.0;
  double sin_lat_ref = std::sin(lat_ref);
  double cos_lat_ref = std::cos(lat_ref);

  double N_ref = a / std::sqrt(1 - e2 * sin_lat_ref * sin_lat_ref);
  double dlat = lat - lat_ref;
  double dlon = lon - lon_ref;

  double NED_N = dlat * N_ref;
  double NED_E = dlon * N_ref * cos_lat_ref;
  double NED_D = LLH[2] - ref_LLH[2];
  return {NED_N, NED_E, NED_D};
}

// --- entrypoint ---
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosUnreal>());
  rclcpp::shutdown();
  return 0;
}
