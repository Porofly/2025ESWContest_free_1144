#include "ImagePublisher.h"
#include <chrono>
#include <thread>
#include <string>
#include <vector>

ImagePublisher::ImagePublisher()
  : Node("image_publisher"),
    fail_count_(0),
    reconnect_requested_(false),
    stop_reading_(false)
{
  RCLCPP_INFO(this->get_logger(), "[INIT] ImagePublisher node 생성 시작");

  // Declare and initialize parameters.
  this->declare_parameter<std::string>("rtsp_url", "rtsp://192.168.10.25:8554/main.264");
  this->declare_parameter<int>("latency", 100);
  this->declare_parameter<std::string>("protocols", "tcp");
  this->declare_parameter<std::string>("topic_name", "compressed_image_topic");
  this->declare_parameter<int>("fps", 10);
  this->declare_parameter<int>("queue_size", 20);
  this->declare_parameter<bool>("leaky_downstream", true);
  this->declare_parameter<int>("jpeg_quality", 50);
  this->declare_parameter<bool>("use_compression", true);
  use_compression_ = this->get_parameter("use_compression").as_bool();

  rtsp_url_ = this->get_parameter("rtsp_url").as_string();
  int latency = this->get_parameter("latency").as_int();
  std::string protocols = this->get_parameter("protocols").as_string();
  topic_name_ = this->get_parameter("topic_name").as_string();
  int publish_fps = this->get_parameter("fps").as_int();
  int queue_size = this->get_parameter("queue_size").as_int();
  bool leaky = this->get_parameter("leaky_downstream").as_bool();
  jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
  std::string leaky_option = leaky ? "downstream" : "no";

  // Build the GStreamer pipeline string.
  pipeline_ =
    "rtspsrc location=rtsp://192.168.10.25:8554/main.264 protocols=tcp "
    "! rtpjitterbuffer "
    "! decodebin "
    "! nvvidconv "
    "! video/x-raw,format=BGRx "
    "! videoconvert "
    "! video/x-raw,format=BGR "
    "! appsink sync=false drop=true max-buffers=1";


  RCLCPP_INFO(this->get_logger(), "[INIT] GStreamer Pipeline 설정: %s", pipeline_.c_str());

  RCLCPP_INFO(this->get_logger(), "[INIT] Pipeline open 시도");
  if (!openPipeline()) {
    RCLCPP_ERROR(this->get_logger(), "[INIT] 초기 Pipeline open 실패");
  } else {
    RCLCPP_INFO(this->get_logger(), "[INIT] 초기 Pipeline open 성공");
  }

  // Create publisher based on the use_compression flag.
  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  if (use_compression_) {
    compressed_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_name_, qos_profile);
    RCLCPP_INFO(this->get_logger(), "[INIT] CompressedImage Publisher 생성 (topic: %s)", topic_name_.c_str());
  } else {
    raw_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("raw_image_topic", qos_profile);
    RCLCPP_INFO(this->get_logger(), "[INIT] Raw Image Publisher 생성 (topic: raw_image_topic)");
  }

  // Start the frame reading thread.
  reading_thread_ = std::thread(&ImagePublisher::readLoop, this);
  RCLCPP_INFO(this->get_logger(), "[INIT] Frame reading thread 시작");

  // Set up timers
  int period_ms = static_cast<int>(1000 / publish_fps);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(period_ms),
    std::bind(&ImagePublisher::timer_callback, this)
  );
  RCLCPP_INFO(this->get_logger(), "[INIT] Publish timer 시작 (주기: %d ms)", period_ms);

  reconnect_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&ImagePublisher::reconnect_callback, this)
  );
  RCLCPP_INFO(this->get_logger(), "[INIT] Reconnect timer 시작 (주기: 1초)");
}

ImagePublisher::~ImagePublisher() {
  RCLCPP_INFO(this->get_logger(), "[DEST] ImagePublisher 종료 중");
  stop_reading_ = true;
  if (reading_thread_.joinable()) {
    reading_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "[DEST] Reading thread 종료 완료");
}

bool ImagePublisher::openPipeline() {
  RCLCPP_INFO(this->get_logger(), "[PIPELINE] Pipeline open 시도");
  if (cap_.isOpened()) {
    RCLCPP_INFO(this->get_logger(), "[PIPELINE] 기존 pipeline 닫기");
    cap_.release();
  }
  cap_.open(pipeline_, cv::CAP_GSTREAMER);
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "[PIPELINE] Pipeline open 실패");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "[PIPELINE] Pipeline open 성공");
  fail_count_ = 0;
  return true;
}

void ImagePublisher::readLoop() {
  RCLCPP_INFO(this->get_logger(), "[READLOOP] Frame 읽기 루프 시작");
  while (rclcpp::ok() && !stop_reading_) {
    if (!cap_.isOpened()) {
      RCLCPP_WARN(this->get_logger(), "[READLOOP] Pipeline이 열려있지 않음, 대기 중...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    cv::Mat frame;
    if (cap_.read(frame)) {
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = frame;
      }
      fail_count_ = 0;
      RCLCPP_INFO(this->get_logger(), "[READLOOP] Frame 수신 성공 (size: %dx%d)", frame.cols, frame.rows);
    } else {
      fail_count_++;
      RCLCPP_WARN(this->get_logger(), "[READLOOP] Frame 읽기 실패 (fail_count=%d)", fail_count_);
      if (fail_count_ >= 10) {
        RCLCPP_WARN(this->get_logger(), "[READLOOP] 실패 횟수 초과 -> Pipeline 재연결 요청");
        reconnect_requested_ = true;
        fail_count_ = 0;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  RCLCPP_INFO(this->get_logger(), "[READLOOP] Frame 읽기 루프 종료");
}

void ImagePublisher::timer_callback() {
  cv::Mat frame_to_publish;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (latest_frame_.empty()) {
      RCLCPP_WARN(this->get_logger(), "[TIMER] 최신 frame 없음 -> Publish 스킵");
      return;
    }
    frame_to_publish = latest_frame_.clone();
  }
  if (use_compression_) {
    //RCLCPP_INFO(this->get_logger(), "[TIMER] JPEG 압축 후 Publish 준비");
    std::vector<uchar> encoded_buf;
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, jpeg_quality_ };
    if (!cv::imencode(".jpg", frame_to_publish, encoded_buf, params)) {
      RCLCPP_ERROR(this->get_logger(), "[TIMER] JPEG 인코딩 실패");
      return;
    }
    sensor_msgs::msg::CompressedImage compressed_msg;
    compressed_msg.header.stamp = now();
    compressed_msg.format = "jpeg";
    compressed_msg.data = encoded_buf;
    compressed_publisher_->publish(compressed_msg);
    //RCLCPP_INFO(this->get_logger(), "[TIMER] Compressed frame publish 완료 (size=%zu bytes)", encoded_buf.size());
  } else {
    RCLCPP_INFO(this->get_logger(), "[TIMER] Raw frame Publish 준비");
    sensor_msgs::msg::Image raw_msg;
    raw_msg.header.stamp = now();
    raw_msg.height = frame_to_publish.rows;
    raw_msg.width = frame_to_publish.cols;
    raw_msg.encoding = "bgr8";
    raw_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_to_publish.step);
    raw_msg.data.assign(frame_to_publish.datastart, frame_to_publish.dataend);
    raw_publisher_->publish(raw_msg);
    RCLCPP_INFO(this->get_logger(), "[TIMER] Raw frame publish 완료 (size=%zu bytes)", raw_msg.data.size());
  }
}

void ImagePublisher::reconnect_callback() {
  if (reconnect_requested_ && rclcpp::ok()) {
    RCLCPP_WARN(this->get_logger(), "[RECONNECT] Pipeline 재연결 시도");
    reconnect_requested_ = false;
    if (!openPipeline()) {
      RCLCPP_ERROR(this->get_logger(), "[RECONNECT] Pipeline 재연결 실패");
    } else {
      RCLCPP_INFO(this->get_logger(), "[RECONNECT] Pipeline 재연결 성공");
    }
  }
}