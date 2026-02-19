#include "angus_depth_image_transport/angus_depth_subscriber.hpp"

#include "angus_depth_image_transport/codec.hpp"

namespace angus_depth_image_transport {

void AngusDepthSubscriber::internalCallback(
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr& message, const Callback& user_cb) {
  sensor_msgs::msg::Image::SharedPtr image = decodeCompressedDepthImage(*message);

  if (image) {
    user_cb(image);
  }
}

}  // namespace angus_depth_image_transport
