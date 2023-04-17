#include "angus_depth_image_transport/angus_depth_publisher.hpp"

#include "angus_depth_image_transport/codec.hpp"

namespace angus_depth_image_transport {

void AngusDepthPublisher::advertiseImpl(rclcpp::Node* node, const std::string& base_topic,
                                        rmw_qos_profile_t custom_qos) {
  typedef image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> Base;
  Base::advertiseImpl(node, base_topic, custom_qos);

  node->get_parameter_or<double>("depth_max", config_.depth_max, 50.0);
  node->get_parameter_or<double>("depth_quantization", config_.depth_quantization, 100.0);
  node->get_parameter_or<int>("png_level", config_.png_level, 1);
}

void AngusDepthPublisher::publish(const sensor_msgs::msg::Image& message,
                                  const PublishFn& publish_fn) const {
  auto logger = rclcpp::get_logger("angus_depth_image_transport");
  RCLCPP_INFO_ONCE(logger, "AngusDepthPub: depth_max: %f", config_.depth_max);
  RCLCPP_INFO_ONCE(logger, "AngusDepthPub: depth_quantization: %f", config_.depth_quantization);
  RCLCPP_INFO_ONCE(logger, "AngusDepthPub: png_level: %i", config_.png_level);

  sensor_msgs::msg::CompressedImage::SharedPtr compressed_image = encodeCompressedDepthImage(
      message, config_.depth_max, config_.depth_quantization, config_.png_level);

  if (compressed_image) {
    publish_fn(*compressed_image);
  }
}

}  // namespace angus_depth_image_transport
