#ifndef IMAGE_PUBLISHER_H_
#define IMAGE_PUBLISHER_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>

/**
 * @brief Node to capture an RTSP stream and publish images.
 *
 * Publishes either compressed (JPEG) or raw images based on the 'use_compression' parameter.
 */
class ImagePublisher : public rclcpp::Node {
public:
  ImagePublisher();
  virtual ~ImagePublisher();

private:
  bool openPipeline();       // Opens the RTSP (GStreamer) pipeline.
  void readLoop();           // Continuously reads frames in a separate thread.
  void timer_callback();     // Publishes the latest captured frame.
  void reconnect_callback(); // Reopens the pipeline upon consecutive failures.

  cv::VideoCapture cap_;
  std::string pipeline_;
  std::string rtsp_url_;
  std::string topic_name_;

  // Publishers for compressed and raw images.
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  std::thread reading_thread_;
  std::mutex frame_mutex_;
  cv::Mat latest_frame_;

  int fail_count_;
  std::atomic<bool> reconnect_requested_;
  std::atomic<bool> stop_reading_;

  int jpeg_quality_;
  bool use_compression_; // Determines whether to publish compressed or raw images.
};

#endif 