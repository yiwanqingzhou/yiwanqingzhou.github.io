---
title: PCL类型转换
date: 2019-09-04 11:32:48
tags:
- pcl
- c++
categories:
- pcl
---



#### PCL相关类型 对象与指针互换

1. pcl::PointIndices -> pcl::PointIndices::Ptr

```c++
pcl::PointIndices inliers;
pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(inliers));
```



2. pcl::PointIndices::Ptr -> pcl::PointIndices

```c++
pcl::PointIndices inliers;
pcl::PointIndices::Ptr inliers_ptr;
inliers=*inliers_ptr;
```



3. std::vector<int> -> pcl::IndicesPtr

```c++
std::vector<int> vec_indices;
pcl::IndicesPtr indices_ptr = boost::make_shared<std::vector<int>>(vec_indices);
```



4. pcl::PointCloud<PointT> -> pcl::PointCloud<PointT>::Ptr

```c++
PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
PointCloud<PointT> cloud;
cloud=*cloud_ptr;
```



5. pcl::PointCloud<PointT>::Ptr -> pcl::PointCloud<PointT>

```c++
PointCloud<PointT>::Ptr cloud_ptr(new pcl::PointCloud<PointT>);
PointCloud<PointT> cloud;
cloud_ptr=cloud.makeShared();
```





#### 不同数据类型的点云转换

1.  pcl::PointCloud<pcl::PointXYZRGB> -> pcl::PointCloud<pcl::PointXYZ>::Ptr

```c++
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::copyPointCloud(*cloud_ori, *cloud);
```

