#include <image_transport/simple_publisher_plugin.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace angus_depth_image_transport {

class AngusDepthPublisher
    : public image_transport::SimplePublisherPlugin<sensor_msgs::msg::CompressedImage> {
 public:
  virtual ~AngusDepthPublisher() {}

  virtual std::string getTransportName() const override final { return "angusDepth"; }

 protected:
  // Overridden to set up reconfigure server
  virtual void advertiseImpl(rclcpp::Node* node, const std::string& base_topic,
                             rmw_qos_profile_t custom_qos) override final;

  virtual void publish(const sensor_msgs::msg::Image& message,
                       const PublishFn& publish_fn) const override final;

  struct Config {
    int png_level;
    double depth_max;
    double depth_quantization;
  };

  Config config_;
};

}  // namespace angus_depth_image_transport
