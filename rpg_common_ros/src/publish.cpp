// Copyright (C) 2017-2018 Titus Cieslewski, RPG, University of Zurich,
//   Switzerland
//   You can contact the author at <titus at ifi dot uzh dot ch>
// Copyright (C) 2017-2018 Davide Scaramuzza, RPG, University of Zurich, 
//   Switzerland
//
// This file is part of dslam_open.
//
// dslam_open is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// dslam_open is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with dslam_open. If not, see <http://www.gnu.org/licenses/>.

#include "rpg_common_ros/publish.h"

#include <opencv2/opencv.hpp>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <minkindr_conversions/kindr_msg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <rpg_common/pose.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

namespace rpg_common_ros {

namespace publish_internal {

template <typename MessageType>
ros::Publisher& getPublisher(const std::string& topic_name)
{
  typedef std::unordered_map<std::string, ros::Publisher> PublisherMap;
  static PublisherMap publishers;
  PublisherMap::iterator found = publishers.find(topic_name);
  if (found == publishers.end())
  {
    found = publishers.emplace(
        topic_name, ros::NodeHandle().advertise<MessageType>(
            topic_name, 5)).first;
  }
  return found->second;
}

std::string world_frame = "world";

}  // namespace publish_internal

// =======================
// PUBLISH SPECIALIZATIONS
// =======================
template <>
void publish(const std::string& topic, const cv::Mat& object)
{
  // Create publisher if necessary.
  typedef std::unordered_map<std::string, image_transport::Publisher>
  PublisherMap;
  static PublisherMap image_publishers;
  PublisherMap::iterator found = image_publishers.find(topic);
  if (found == image_publishers.end())
  {
    found = image_publishers.emplace(topic, image_transport::ImageTransport(
        ros::NodeHandle()).advertise(topic, 10)).first;
  }

  cv_bridge::CvImage image_message;
  image_message.image = object;
  switch (object.type()) {
    case CV_8UC1:
      image_message.encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case CV_8UC3:
      image_message.encoding = sensor_msgs::image_encodings::RGB8;
      break;
    default:
      LOG(FATAL) << "Image encoding for type " << object.type() <<
      " not determined automatically. Consider adding a rule.";
      break;
  }
  found->second.publish(image_message.toImageMsg());
}

template <>
void publish(const std::string& topic, const geometry_msgs::PoseStamped& object)
{
  publish_internal::getPublisher<geometry_msgs::PoseStamped>(topic).publish(
      object);
}

template <>
void publish(const std::string& topic, const rpg_common::Pose& object,
             const std::string& reference_frame)
{
  geometry_msgs::PoseStamped pose_message;
  tf::poseStampedKindrToMsg(object, reference_frame, &pose_message);
  publish(topic, pose_message);
}

template <>
void publish(const std::string& topic, const visualization_msgs::Marker& marker)
{
  publish_internal::getPublisher<visualization_msgs::Marker>(topic).publish(
      marker);
}

template <>
void publish(const std::string& topic, const sensor_msgs::PointCloud2& cloud)
{
  publish_internal::getPublisher<sensor_msgs::PointCloud2>(topic).publish(
      cloud);
}

template <>
void publish(
    const std::string& topic, const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  publish_internal::getPublisher<sensor_msgs::PointCloud2>(topic).publish(
      cloud_msg);
}

template <>
void publish(
    const std::string& topic, const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  publish_internal::getPublisher<sensor_msgs::PointCloud2>(topic).publish(
      cloud_msg);
}

// ============================
// PUBLISH SPECIALIZATIONS DONE
// ============================

void publishTf(const Eigen::Matrix4d& T_A_B, const ros::Time& ros_time,
        const std::string& A_name, const std::string& B_name)
{
  const tf::Matrix3x3 R_A_B(T_A_B(0, 0), T_A_B(0, 1), T_A_B(0, 2),
                            T_A_B(1, 0), T_A_B(1, 1), T_A_B(1, 2),
                            T_A_B(2, 0), T_A_B(2, 1), T_A_B(2, 2));
  const tf::Vector3 p_A_B(T_A_B(0, 3), T_A_B(1, 3), T_A_B(2, 3));
  tf::Transform transform_message(R_A_B, p_A_B);
  static tf::TransformBroadcaster broadcaster;
  broadcaster.sendTransform(tf::StampedTransform(
      transform_message, ros_time, A_name, B_name));
}

}  // namespace rpg_common_ros
