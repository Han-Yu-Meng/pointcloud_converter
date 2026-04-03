/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

// util_funcs.hpp

#pragma once

#include <fins/functional_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <livox_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using fins::Function;
using fins::Input;
using fins::Output;
using fins::Parameter;

static auto livox_lidar_convert_node = Function("Livox2PCL",
  [](Input<livox_driver2::msg::CustomMsg> &lidar_msg,
    Output<pcl::PointCloud<pcl::PointXYZI>::Ptr> &cloud_out) {
    if (lidar_msg->point_num == 0) return;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->width = lidar_msg->point_num;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    for (size_t i = 0; i < lidar_msg->point_num; ++i) {
      pcl::PointXYZI point;
      point.x = lidar_msg->points[i].x;
      point.y = lidar_msg->points[i].y;
      point.z = lidar_msg->points[i].z;
      point.intensity = static_cast<float>(lidar_msg->points[i].reflectivity);
      cloud->points[i] = point;
    }
    cloud_out = cloud;
  }
  ).with_description("Converts Livox CustomMsg to PCL PointCloud.")
   .with_inputs_description({"livox_cloud"})
   .with_outputs_description({"pcl_cloud"})
   .with_category("Point Cloud>Conversion")
   .build();


static auto livox_lidar_ros_convert_node = Function("Livox2ROS",
  [](Input<livox_driver2::msg::CustomMsg> &cloud_in,
    Output<sensor_msgs::msg::PointCloud2> &lidar_msg) {
    if (cloud_in->point_num == 0) return;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.width = cloud_in->point_num;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.points.resize(cloud.width * cloud.height);
    for (size_t i = 0; i < cloud_in->point_num; ++i) {
      pcl::PointXYZI point;
      point.x = cloud_in->points[i].x;
      point.y = cloud_in->points[i].y;
      point.z = cloud_in->points[i].z;
      point.intensity = static_cast<float>(cloud_in->points[i].reflectivity);
      cloud.points[i] = point;
    }
    pcl::toROSMsg(cloud, *lidar_msg);
    lidar_msg->header = cloud_in->header;
  }
  ).with_description("Converts Livox CustomMsg to ROS PointCloud2 message.")
   .with_inputs_description({"livox_cloud"})
   .with_outputs_description({"ros_cloud"})
   .with_category("Point Cloud>Conversion")
   .build();


static auto ros_lidar_convert_node = Function("ROS2PCL",
  [](Input<sensor_msgs::msg::PointCloud2> &lidar_msg,
    Output<pcl::PointCloud<pcl::PointXYZI>::Ptr> &cloud_out) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::fromROSMsg(*lidar_msg, *cloud);
    cloud_out = cloud;
    (*cloud_out)->header = pcl_conversions::toPCL(lidar_msg->header);
  }
  ).with_description("Converts ROS PointCloud2 message to PCL PointCloud.")
   .with_inputs_description({"ros_cloud"})
   .with_outputs_description({"pcl_cloud"})
   .with_category("Point Cloud>Conversion")
   .build();

static auto pcl_ros_convert_node = Function("PCL2ROS",
  [](Input<pcl::PointCloud<pcl::PointXYZI>::Ptr> &cloud_in,
    Output<sensor_msgs::msg::PointCloud2> &lidar_msg) {
    if (!*cloud_in) return;
    pcl::toROSMsg(**cloud_in, *lidar_msg);
    lidar_msg->header = pcl_conversions::fromPCL((*cloud_in)->header);
  }
  ).with_description("Converts PCL PointCloud to ROS PointCloud2 message.")
   .with_inputs_description({"pcl_cloud"})
   .with_outputs_description({"ros_cloud"})
   .with_category("Point Cloud>Conversion")
   .build();

static auto pcl_rgb_ros_convert_node = Function("PCLRGB2ROS",
  [](Input<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_in,
    Output<sensor_msgs::msg::PointCloud2> &lidar_msg) {
    if (!*cloud_in) return;
    pcl::toROSMsg(**cloud_in, *lidar_msg);
    lidar_msg->header = pcl_conversions::fromPCL((*cloud_in)->header);
  }
  ).with_description("Converts PCL PointCloud<PointXYZRGB> to ROS PointCloud2 message.")
   .with_inputs_description({"pcl_cloud"})
   .with_outputs_description({"ros_cloud"})
   .with_category("Point Cloud>Conversion")
   .build();

static auto ros_lidar_rgb_convert_node = Function("ROS2PCLRGB",
  [](Input<sensor_msgs::msg::PointCloud2> &lidar_msg,
    Output<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &cloud_out) {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::fromROSMsg(*lidar_msg, *cloud);
    cloud_out = cloud;
    (*cloud_out)->header = pcl_conversions::toPCL(lidar_msg->header);
  }
  ).with_description("Converts ROS PointCloud2 message to PCL PointCloud<PointXYZRGB>.")
   .with_inputs_description({"ros_cloud"})
   .with_outputs_description({"pcl_cloud"})
   .with_category("Point Cloud>Conversion")
   .build();

static auto pcl_normal_to_intensity_convert_node = Function("NormalToIntensity",
  [](Input<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> &cloud_in,
    Output<pcl::PointCloud<pcl::PointXYZI>::Ptr> &cloud_out) {
    if (!*cloud_in) return;
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->width = (*cloud_in)->width;
    cloud->height = (*cloud_in)->height;
    cloud->is_dense = (*cloud_in)->is_dense;
    cloud->points.resize((*cloud_in)->points.size());
    cloud->header = (*cloud_in)->header;
    for (size_t i = 0; i < (*cloud_in)->points.size(); ++i) {
      cloud->points[i].x = (*cloud_in)->points[i].x;
      cloud->points[i].y = (*cloud_in)->points[i].y;
      cloud->points[i].z = (*cloud_in)->points[i].z;
      cloud->points[i].intensity = (*cloud_in)->points[i].intensity;
    }
    cloud_out = cloud;
  }
  ).with_description("Converts PCL PointCloud<PointXYZINormal> to PointCloud<PointXYZI>.")
  .with_inputs_description({"normal_cloud"})
  .with_outputs_description({"intensity_cloud"})
  .with_category("Point Cloud>Conversion")
  .build();