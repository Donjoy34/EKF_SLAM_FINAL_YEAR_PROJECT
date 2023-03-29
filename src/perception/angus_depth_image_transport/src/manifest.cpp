#include <pluginlib/class_list_macros.hpp>

#include "angus_depth_image_transport/angus_depth_publisher.hpp"
#include "angus_depth_image_transport/angus_depth_subscriber.hpp"

PLUGINLIB_EXPORT_CLASS(angus_depth_image_transport::AngusDepthPublisher,
                       image_transport::PublisherPlugin)

PLUGINLIB_EXPORT_CLASS(angus_depth_image_transport::AngusDepthSubscriber,
                       image_transport::SubscriberPlugin)
