#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

class CompressedImageSubscriber : public rclcpp::Node {
public:
  CompressedImageSubscriber()
  : Node("compressed_image_subscriber")
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "compressed_image_topic", qos_profile,
      std::bind(&CompressedImageSubscriber::callback, this, std::placeholders::_1)
    );
  }
  void callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
      cv::Mat encoded(1, msg->data.size(), CV_8UC1, (void*)msg->data.data());
      cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
      if (!decoded.empty()) {
        // 디코딩된 이미지가 정상적이면, 뮤텍스를 이용해 latest_frame_에 복사 저장
        std::lock_guard<std::mutex> lock(mutex_);
        latest_frame_ = decoded; // 여기서 이미지 복사
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to decode compressed image");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception in callback: %s", e.what());
    }
  }
  cv::Mat getLatestFrame() {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_frame_.clone();
  }

private:
  // 압축된 이미지 메시지를 구독하기 위한 Subscription 객체
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  // 디코딩된 최신 프레임을 저장할 cv::Mat 객체
  cv::Mat latest_frame_;
  // 멀티스레드 환경에서 latest_frame_ 접근을 보호하기 위한 뮤텍스
  std::mutex mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CompressedImageSubscriber>();
  cv::namedWindow("Compressed Image", cv::WINDOW_NORMAL);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    cv::Mat frame = node->getLatestFrame();
    if (!frame.empty()) {
      cv::imshow("Compressed Image", frame);
    }
    cv::waitKey(1);
  }
  rclcpp::shutdown();
  return 0;
}