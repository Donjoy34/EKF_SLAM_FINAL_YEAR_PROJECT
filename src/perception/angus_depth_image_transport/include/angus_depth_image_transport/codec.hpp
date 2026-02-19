#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

// Encoding and decoding of compressed depth images.
namespace angus_depth_image_transport {

// Returns a null pointer on bad input.
sensor_msgs::msg::Image::SharedPtr decodeCompressedDepthImage(
    const sensor_msgs::msg::CompressedImage& compressed_image);

// Compress a depth image. The png_compression parameter is passed straight through to
// OpenCV as IMWRITE_PNG_COMPRESSION. Returns a null pointer on bad input.
sensor_msgs::msg::CompressedImage::SharedPtr encodeCompressedDepthImage(
    const sensor_msgs::msg::Image& message, double depth_max = 10.0,
    double depth_quantization = 100.0, int png_level = 1);
}  // namespace angus_depth_image_transport
