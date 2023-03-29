#include "angus_depth_image_transport/codec.hpp"

#include <cv_bridge/cv_bridge.h>

#include <limits>
//#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <vector>

#include "angus_depth_image_transport/compression_common.hpp"

namespace enc = sensor_msgs::image_encodings;

// Encoding and decoding of compressed depth images.
namespace angus_depth_image_transport {

sensor_msgs::msg::Image::SharedPtr decodeCompressedDepthImage(
    const sensor_msgs::msg::CompressedImage& message) {
  if (message.data.size() <= sizeof(ConfigHeader)) {
    return nullptr;
  }

  // Get ROS logger
  auto logger = rclcpp::get_logger("angus_depth_image_transport");

  // CV Image
  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  cv_ptr->header = message.header;
  cv_ptr->encoding = message.format.substr(0, message.format.find(';'));
  if (enc::bitDepth(cv_ptr->encoding) != 32) {
    return nullptr;
  }

  // Read compression type from stream
  ConfigHeader compressionConfig;
  memcpy(&compressionConfig, &message.data[0], sizeof(ConfigHeader));

  // Get compressed image data
  const std::vector<uint8_t> imageData(message.data.begin() + sizeof(ConfigHeader),
                                       message.data.end());

  cv::Mat decompressed;
  try {
    // Decode image data
    decompressed = cv::imdecode(imageData, cv::IMREAD_UNCHANGED);
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    return sensor_msgs::msg::Image::SharedPtr();
  }

  size_t rows = decompressed.rows;
  size_t cols = decompressed.cols;
  if (rows <= 0 || cols <= 0) {
    return nullptr;
  }

  // Depth conversion
  cv_ptr->image = cv::Mat(rows, cols, CV_32FC1);
  cv::MatIterator_<float> itDepthImg = cv_ptr->image.begin<float>();
  cv::MatIterator_<float> itDepthImg_end = cv_ptr->image.end<float>();
  cv::MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>();
  cv::MatConstIterator_<unsigned short> itInvDepthImg_end = decompressed.end<unsigned short>();

  // Depth map decoding
  float depthQuantA = compressionConfig.depthParam[0];
  float depthQuantB = compressionConfig.depthParam[1];

  for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end);
       ++itDepthImg, ++itInvDepthImg) {
    // check for NaN & max depth
    if (*itInvDepthImg) {
      *itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
    } else {
      // RCLCPP_ERROR(logger, "itInvDepthImg has value of false: %f", *itDepthImg);
      *itDepthImg = std::numeric_limits<float>::quiet_NaN();
    }
  }

  // cv::imshow("image", cv_ptr.get()->image);
  // cv::waitKey(10);

  // Publish message to user callback
  return cv_ptr->toImageMsg();
}

sensor_msgs::msg::CompressedImage::SharedPtr encodeCompressedDepthImage(
    const sensor_msgs::msg::Image& message, double depth_max, double depth_quantization,
    int png_level) {
  // Get ROS logger
  auto logger = rclcpp::get_logger("angus_depth_image_transport");
  logger.set_level(rclcpp::Logger::Level::Debug);

  // Compressed image message
  sensor_msgs::msg::CompressedImage::SharedPtr compressed(new sensor_msgs::msg::CompressedImage());
  compressed->header = message.header;
  compressed->format = message.encoding + "; compressedDepth";

  // Compression settings
  // std::vector<int> params(3, 0);
  std::vector<int> params(2, 0);
  params[0] = cv::IMWRITE_PNG_COMPRESSION;
  params[1] = png_level;

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);
  int numChannels = enc::numChannels(message.encoding);
  RCLCPP_DEBUG_ONCE(logger, "bitDepth: %i", bitDepth);
  RCLCPP_DEBUG_ONCE(logger, "numChannels: %i", numChannels);
  if (bitDepth != 32 || numChannels != 1) {
    return nullptr;
  }

  // OpenCV-ROS bridge
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(message);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(logger, "%s", e.what());
    return sensor_msgs::msg::CompressedImage::SharedPtr();
  }

  // cv::imshow("image", cv_ptr.get()->image);
  // cv::waitKey(10);

  const cv::Mat& depthImg = cv_ptr->image;
  size_t rows = depthImg.rows;
  size_t cols = depthImg.cols;
  if (rows <= 0 || cols <= 0) {
//    return nullptr;
  }

  // Allocate matrix for inverse depth (disparity) coding
  cv::Mat invDepthImg(rows, cols, CV_16UC1);

  // Matrix iterators
  cv::MatConstIterator_<float> itDepthImg = depthImg.begin<float>();
  cv::MatConstIterator_<float> itDepthImg_end = depthImg.end<float>();
  cv::MatIterator_<unsigned short> itInvDepthImg = invDepthImg.begin<unsigned short>();
  cv::MatIterator_<unsigned short> itInvDepthImg_end = invDepthImg.end<unsigned short>();

  // Inverse depth quantization parameters
  float depthZ0 = depth_quantization;
  float depthMax = depth_max;
  float depthQuantA = depthZ0 * (depthZ0 + 1.0f);
  float depthQuantB = 1.0f - depthQuantA / depthMax;

  // Quantization
  for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end);
       ++itDepthImg, ++itInvDepthImg) {
    // check for NaN & max depth
    if (*itDepthImg < depthMax) {
      // RCLCPP_ERROR(logger, "itDepthImg is fucking terrible: %f", *itDepthImg);
      *itInvDepthImg = (depthQuantA / *itDepthImg) + depthQuantB;
    } else {
      *itInvDepthImg = 0;
    }
  }

  // Compressed image data
  std::vector<uint8_t> compressedImage;
  try {
    // Compress quantized disparity image
    if (cv::imencode(".png", invDepthImg, compressedImage, params)) {
      float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) /
                     (float)compressedImage.size();
      RCLCPP_DEBUG(logger, "Compressed Depth Image Transport - Compression: 1:%.2f (%lu bytes)",
                   cRatio, compressedImage.size());
    } else {
      RCLCPP_ERROR(logger, "cv::imencode (png) failed on input image");
      return sensor_msgs::msg::CompressedImage::SharedPtr();
    }
  } catch (cv::Exception& e) {
    RCLCPP_ERROR(logger, "%s", e.msg.c_str());
    return sensor_msgs::msg::CompressedImage::SharedPtr();
  }

  if (compressedImage.size() > 0) {
    // Image compression configuration
    ConfigHeader compressionConfig;
    compressionConfig.format = INV_DEPTH;
    compressionConfig.depthParam[0] = depthQuantA;
    compressionConfig.depthParam[1] = depthQuantB;

    // Add configuration to binary output
    compressed->data.resize(sizeof(ConfigHeader));
    memcpy(&compressed->data[0], &compressionConfig, sizeof(ConfigHeader));

    // Add compressed binary data to messages
    compressed->data.insert(compressed->data.end(), compressedImage.begin(), compressedImage.end());

    return compressed;
  } else {
    RCLCPP_ERROR(logger, "compressedImage size <= 0!");
  }

  return sensor_msgs::msg::CompressedImage::SharedPtr();
}

}  // namespace angus_depth_image_transport
