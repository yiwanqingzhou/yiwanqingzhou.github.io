---
title: ROS的PCL点云相关类型转换
date: 2025-08-15 18:57:54
tags:
- pcl
- c++
- linux
- ros
- cloud
- pointcloud
categories:
- [pcl]
- [c++]
keywords:
description:
---

`ROS` 中使用 [sensor_msgs::msg::PointCloud2](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) 传输点云数据。开发中会需要对它和其他类型的点云数据进行转换，大部分由 [pcl_conversions](https://github.com/ros-perception/pcl_conversions) 提供。

### pcl::PCLPointCloud2

ros msg -> pcl::PCLPointCloud2

```C++
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

pcl::PCLPointCloud2 pcl_pc2;

sensor_msgs::msg::PointCloud2 ros_cloud;
pcl_conversions::toPCL(ros_cloud, pcl_pc2);

sensor_msgs::msg::PointCloud2::SharedPtr pc_msg;
pcl_conversions::toPCL(*pc_msg.get(), pcl_pc2);
```

ros msg <- pcl::PCLPointCloud2

```C++
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

pcl::PCLPointCloud2 pcl_pc2;

sensor_msgs::msg::PointCloud2 ros_cloud;
pcl_conversions::fromPCL(pcl_pc2, ros_cloud);

sensor_msgs::msg::PointCloud2::SharedPtr pc_msg;
pcl_conversions::fromPCL(pcl_pc2, *pc_msg);
```

### pcl::PointCloud<T>

ros msg -> pcl::PointCloud<T>

```C++

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;

sensor_msgs::msg::PointCloud2 ros_cloud;
pcl::fromROSMsg(ros_cloud, *cloud_ptr);

sensor_msgs::msg::PointCloud2::SharedPtr pc_msg;
pcl::fromROSMsg(*pc_msg.get(), *cloud_ptr);
```

ros msg <- pcl::PointCloud<T>

```C++
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr;
if (!cloud_ptr || cloud_ptr->points.empty())
  return;

sensor_msgs::msg::PointCloud2 ros_cloud;
pcl::toROSMsg(*cloud_ptr, ros_cloud);

sensor_msgs::msg::PointCloud2::SharedPtr pc_msg;
pcl::toROSMsg(*cloud_ptr, *pc_msg);
```
