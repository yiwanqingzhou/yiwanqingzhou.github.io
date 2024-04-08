---
title: PointCloud<PointT> 和 PCLPointCloud2 的区别
date: 2024-04-08 16:16:43
tags:
- pcl
- ros
categories:
- pcl
keywords:
- pcl
- pointcloud
- 点云
- ros
description:
---



`PCL` 中有两种表示点云的数据结构，分别为 `PointCloud<PointT>` 和 `PCLPointCloud2`。官方注释中常称为 `a pcl::PointCloud<T> object` 以及 `a PCLPointCloud2 binary data blob`。



两者的最大区别是**储存数据的方式**：

- `PointCloud<PointT>` 为模板类，其中指定了每个点的数据类型 `PointT`， 独立储存每个点的数据。这种存储方式使得数据非常清晰，可以很方便地对某一个点或是某个点的某一字段进行访问，但无法选择存储或删除某一字段。

  ```C++
  template <typename PointT>
  class PointCloud {
  public:
  	std::vector<PointT, Eigen::aligned_allocator<PointT>> points;
      ...
  };
  ```
  
- `PCLPointCloud2` 则没有指定点的数据类型，而是在 `fields` 里记录每个点中有哪些字段（比如 `rgba` , `x` , `normal_x` 等），并以 `std::uint8_t` 将它们按顺序连续存储。这种存储方式理论上更通用，能够存储各种类型的点云数据，而不仅是 `PCL`中定义好的常见格式；可以灵活地对数据进行直接处理，选择存储或删除某一字段；当然也使得数据变得不太直观。

  ```C++
  struct PCLPointCloud2{
      std::vector<::pcl::PCLPointField>  fields;
      uindex_t point_step = 0;
      std::vector<std::uint8_t> data;
      ...
  };
  ```

<!-- more -->


`ROS` 中使用 `PCLPointCloud2` 传输点云数据: [sensor_msgs::msg::PointCloud2](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html)



`PCL` 中提供了两者互换的接口：

```C++
template<typename PointT> void
fromPCLPointCloud2 (const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud)

template<typename PointT> void
toPCLPointCloud2 (const pcl::PointCloud<PointT>& cloud, pcl::PCLPointCloud2& msg)

//以前为 `fromROSMsg` 和`toROSMsg`
```

