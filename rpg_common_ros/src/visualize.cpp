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

#include "rpg_common_ros/visualize.h"
#include <vector>

#include <Eigen/Dense>
#include <minkindr_conversions/kindr_msg.h>
#include <pcl_ros/point_cloud.h>
#include <rpg_common/aligned.h>
#include <rpg_common/sim3.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "rpg_common_ros/publish.h"

namespace rpg_common_ros {

namespace visualize_internal {

std_msgs::ColorRGBA createColor(
    const float r, const float g, const float b, const float a)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

std_msgs::ColorRGBA createColor(
    const float r, const float g, const float b)
{
  return createColor(r, g, b, 1);
}

std_msgs::ColorRGBA createColor(const Eigen::Vector3d& color)
{
  return createColor(color(0), color(1), color(2), 1);
}

void addPoint(const Eigen::Vector3d& p_W_A, const std_msgs::ColorRGBA& color,
              visualization_msgs::Marker* marker)
{
  CHECK_NOTNULL(marker);
  geometry_msgs::Point point;
  point.x = p_W_A.x();
  point.y = p_W_A.y();
  point.z = p_W_A.z();
  marker->points.push_back(point);
  marker->colors.push_back(color);
}

void addPoints(const Eigen::Matrix3Xd& p_W_A, const std_msgs::ColorRGBA& color,
               visualization_msgs::Marker* marker)
{
  CHECK_NOTNULL(marker)->points.reserve(p_W_A.cols());
  marker->colors.reserve(p_W_A.cols());
  for (int i = 0; i < p_W_A.cols(); ++i)
  {
    visualize_internal::addPoint(p_W_A.col(i), color, marker);
  }
}

void addPoints(const rpg::Aligned<std::vector, Eigen::Vector3d>& p_W_A , const std_msgs::ColorRGBA& color,
               visualization_msgs::Marker* marker)
{
  CHECK_NOTNULL(marker)->points.reserve(p_W_A.size());
  marker->colors.reserve(p_W_A.size());
  for (const Eigen::Vector3d& point: p_W_A)
  {
    visualize_internal::addPoint(point, color, marker);
  }
}

void addFrame(
    const rpg_common::Sim3& Sim3_W_A, visualization_msgs::Marker* marker)
{
  CHECK_NOTNULL(marker);
  CHECK_EQ(marker->type, visualization_msgs::Marker::LINE_LIST);
  Eigen::Matrix3Xd p_A_points(3, 4);  // origin, x, y, z
  p_A_points.col(0).setZero();
  p_A_points.rightCols(3) = Eigen::Matrix3d::Identity();
  const Eigen::Matrix3Xd p_W_points = Sim3_W_A * p_A_points;

  addPoint(p_W_points.col(0), createColor(1, 0, 0), marker);
  addPoint(p_W_points.col(1), createColor(1, 0, 0), marker);
  addPoint(p_W_points.col(0), createColor(0, 1, 0), marker);
  addPoint(p_W_points.col(2), createColor(0, 1, 0), marker);
  addPoint(p_W_points.col(0), createColor(0, 0, 1), marker);
  addPoint(p_W_points.col(3), createColor(0, 0, 1), marker);
}

void pointsEigenToPointCloud(
    const Eigen::Matrix3Xd& points, const std_msgs::ColorRGBA& color,
    sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  const size_t num_points = static_cast<size_t>(points.cols());

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(num_points);
  for (size_t i = 0u; i < num_points; ++i) {
    pcl::PointXYZRGB point;
    point.x = points(0, i);
    point.y = points(1, i);
    point.z = points(2, i);
    point.r = color.r * 255;
    point.g = color.g * 255;
    point.b = color.b * 255;
    point.a = color.a * 255;
    cloud.push_back(point);
  }

  pcl::toROSMsg(cloud, *point_cloud);
}
void pointsEigenToPointCloud(
    const rpg::Aligned<std::vector, Eigen::Vector3d>& points,
    sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  const size_t num_points = static_cast<size_t>(points.size());

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(num_points);
  for (size_t i = 0u; i < num_points; ++i) {
    pcl::PointXYZ point;
    point.x = points[i](0);
    point.y = points[i](1);
    point.z = points[i](2);
    cloud.push_back(point);
  }

  pcl::toROSMsg(cloud, *point_cloud);
}

}  // namespace visualize_internal

template <>
void visualize(
    const std::string& topic, const Eigen::Matrix3Xd& points, const int& id,
    const PointVisualizationStyle& style)
{
  visualization_msgs::Marker marker;
  publish_internal::setDefaultHeader(&marker.header);
  marker.id = id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.02;

  visualize_internal::addPoints(
      points, visualize_internal::createColor(1, 1, 1), &marker);

  switch (style) {
    case PointVisualizationStyle::kConnectedLine:
    {
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      break;
    }

    case PointVisualizationStyle::kPointCloud:
    {
      marker.type = visualization_msgs::Marker::POINTS;
      break;
    }
  }

  publish(topic, marker);
}

template <>
void visualize(
    const std::string& topic, const rpg::Aligned<std::vector, Eigen::Vector3d>& points, const int& id,
    const PointVisualizationStyle& style, const Eigen::Vector3d& color)
{
  visualization_msgs::Marker marker;
  publish_internal::setDefaultHeader(&marker.header);
  marker.id = id;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.02;

  visualize_internal::addPoints(
      points, visualize_internal::createColor(color), &marker);

  switch (style) {
    case PointVisualizationStyle::kConnectedLine:
    {
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      break;
    }

    case PointVisualizationStyle::kPointCloud:
    {
      marker.type = visualization_msgs::Marker::POINTS;
      break;
    }
  }

  publish(topic, marker);
}

template <>
void visualize(
    const std::string& topic, const Eigen::Matrix3Xd& points)
{
  sensor_msgs::PointCloud2 point_cloud;
  visualize_internal::pointsEigenToPointCloud(
      points, visualize_internal::createColor(1, 1, 1), &point_cloud);
  publish_internal::setDefaultHeader(&point_cloud.header);
  publish(topic, point_cloud);
}

template <>
void visualize(
    const std::string& topic,
    const rpg::Aligned<std::vector, Eigen::Vector3d>& points)
{
  sensor_msgs::PointCloud2 point_cloud;
  visualize_internal::pointsEigenToPointCloud(points, &point_cloud);
  publish_internal::setDefaultHeader(&point_cloud.header);
  publish(topic, point_cloud);
}

template <>
void visualize(
    const std::string& topic, const Eigen::Matrix3Xd& match_from,
    const Eigen::Matrix3Xd& match_to)
{
  CHECK_EQ(match_from.cols(), match_to.cols());

  visualization_msgs::Marker marker;
  publish_internal::setDefaultHeader(&marker.header);
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;

  for (int i = 0; i < match_from.cols(); ++i)
  {
    visualize_internal::addPoint(
        match_from.col(i), visualize_internal::createColor(1, 1, 1), &marker);
    visualize_internal::addPoint(
        match_to.col(i), visualize_internal::createColor(1, 1, 1), &marker);
  }

  publish(topic, marker);
}

template <>
void visualize(
    const std::string& topic,
    const rpg::Aligned<std::vector, Eigen::Vector3d>& match_from,
    const rpg::Aligned<std::vector, Eigen::Vector3d>& match_to)
{
  visualize(topic, rpg::aligned::toMat(match_from),
            rpg::aligned::toMat(match_to));
}

template <>
void visualize(
    const std::string& topic,
    const std::vector<rpg_common::Sim3>& sim3s, const int& id)
{
  visualization_msgs::Marker marker;
  publish_internal::setDefaultHeader(&marker.header);
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.2;

  for (const rpg_common::Sim3& sim3 : sim3s)
  {
    visualize_internal::addFrame(sim3, &marker);
  }

  publish(topic, marker);
}

template <>
void visualize(
    const std::string& topic,
    const std::vector<rpg_common::Sim3>& sim3s)
{
  visualize(topic, sim3s, 0);
}

void visualizeAsSpheres(
    const std::string& topic, const Eigen::Matrix3Xd& centers,
    const Eigen::Vector3d& color, const double radius, const int id)
{
  visualization_msgs::Marker marker;
  publish_internal::setDefaultHeader(&marker.header);
  marker.id = id;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;
  marker.color = visualize_internal::createColor(color);

  visualize_internal::addPoints(
      centers, visualize_internal::createColor(color), &marker);

  publish(topic, marker);
}

void visualizeMesh(
    const std::string& topic, const std::string& mesh_resource,
    const rpg::Pose& pose, const int id)
{
  visualization_msgs::Marker marker;
  publish_internal::setDefaultHeader(&marker.header);
  marker.id = id;
  tf::poseKindrToMsg(pose, &marker.pose);
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_resource;
  marker.mesh_use_embedded_materials = true;
  marker.color = visualize_internal::createColor(0, 0, 0);
  marker.frame_locked = true;

  publish(topic, marker);
}

}  // namespace rpg_common_ros
