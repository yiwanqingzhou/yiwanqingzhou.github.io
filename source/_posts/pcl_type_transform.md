---
title: pcl类型转换
date: 2019-09-04 11:32:48
tags:
- pcl
categories:
- note
---



{% note info %}
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;PCL相关的对象与指针互换
{% endnote %}



##### pcl::PointIndices -> pcl::PointIndices::Ptr

```c++
pcl::PointIndices inliers;
pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(inliers));
```



##### pcl::PointIndices::Ptr -> pcl::PointIndices

```c++
pcl::PointIndices inliers;
pcl::PointIndices::Ptr inliers_ptr;
inliers=*inliers_ptr;
```



##### pcl::PointCloud<PointT> -> pcl::PointCloud<PointT>::Ptr

```C++
PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
PointCloud<PointT> cloud;
cloud=*cloud_ptr;
```



##### pcl::PointCloud<PointT>::Ptr -> pcl::PointCloud<PointT>

```C++
PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
PointCloud<PointT> cloud;
cloud_ptr=cloud.makeShared();
```

