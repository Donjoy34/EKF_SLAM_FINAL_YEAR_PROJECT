#include <image_transport/simple_subscriber_plugin.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace angus_depth_image_transport {

class AngusDepthSubscriber
    : public image_transport::SimpleSubscriberPlugin<sensor_msgs::msg::CompressedImage> {
 public:
  virtual ~AngusDepthSubscriber() {}

  virtual std::string getTransportName() const override final { return "angusDepth"; }

 protected:
  virtual void internalCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& message,
                                const Callback& user_cb) override final;
};

}  // namespace angus_depth_image_transport
