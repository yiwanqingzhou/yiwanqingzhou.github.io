---
title: PCL点云的表示方式
date: 2023-11-23 16:15:47
tags:
- pcl
- cv
- c++
categories:
- pcl
keywords:
- pcl
- 点云
- 点云处理
description:
---

### 点云的表示方式



pcl里的点云多种表示方式：

```c++
// const
ConstCloudIterator<PointT> cloud_iterator;

pcl::PointCloud<PointT> cloud;

// with indices
pcl::PointCloud<PointT> cloud;
std::vector<int> indices;

// with indices
pcl::PointCloud<PointT> cloud;
pcl::PointIndices indices;
```



几乎每个函数都配有这几种表示方式的 `template`，比如 `compute3DCentroid()` 会有对应的以下几种接口：

```c++
template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (ConstCloudIterator<PointT> &cloud_iterator,
                   Eigen::Matrix<Scalar, 4, 1> &centroid);

template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud, 
                   Eigen::Matrix<Scalar, 4, 1> &centroid);

template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   const std::vector<int> &indices, 
                   Eigen::Matrix<Scalar, 4, 1> &centroid);

template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   const pcl::PointIndices &indices, 
                   Eigen::Matrix<Scalar, 4, 1> &centroid);
```

在这个特定例子里，`centroid` 也有多种的表示方式，所以模版数量很多，看起来有些让人眼花缭乱。



但实际上，它们的实现可能非常类似。比如，对比一下点云和带 `indices` 点云的接口的具体实现：

```c++
template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  if (cloud.empty ())
    return (0);

  // Initialize to 0
  centroid.setZero ();
  // For each point in the cloud
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (const auto& point: cloud)
    {
      centroid[0] += point.x;
      centroid[1] += point.y;
      centroid[2] += point.z;
    }
    centroid /= static_cast<Scalar> (cloud.size ());
    centroid[3] = 1;
    return (static_cast<unsigned int> (cloud.size ()));
  }

  // NaN or Inf values could exist => check for them
  unsigned cp = 0;
  for (const auto& point: cloud)
  {
    // Check if the point is invalid
    if (!isFinite (point))
      continue;

    centroid[0] += point.x;
    centroid[1] += point.y;
    centroid[2] += point.z;
    ++cp;
  }
  centroid /= static_cast<Scalar> (cp);
  centroid[3] = 1;

  return (cp);
}
```

```C++
template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   const std::vector<int> &indices,
                   Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  if (indices.empty ())
    return (0);

  // Initialize to 0
  centroid.setZero ();
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (const int& index : indices)
    {
      centroid[0] += cloud[index].x;
      centroid[1] += cloud[index].y;
      centroid[2] += cloud[index].z;
    }
    centroid /= static_cast<Scalar> (indices.size ());
    centroid[3] = 1;
    return (static_cast<unsigned int> (indices.size ()));
  }

  // NaN or Inf values could exist => check for them
  unsigned cp = 0;
  for (const int& index : indices)
  {
    // Check if the point is invalid
    if (!isFinite (cloud [index]))
      continue;

    centroid[0] += cloud[index].x;
    centroid[1] += cloud[index].y;
    centroid[2] += cloud[index].z;
    ++cp;
  }
  centroid /= static_cast<Scalar> (cp);
  centroid[3] = 1;
  return (cp);
}
```

很明显，只有少数的几处有区别：

- 开头判空
  - cloud

  ```c++
  if (cloud.empty()) return 0;
  ```

  - cloud with indices

    ```c++
    if (indices.empty()) return 0;
    ```

- for loop
  - cloud
  
  ```c++
  for (const auto& point: cloud)
  {
    // point
  }
  ```
  
  - cloud with indices
  
  ```c++
  for (const int& index: indices)
  {
    // cloud[index]
  }
  ```



而带 `pcl::PointIndics` 的接口则是直接调用了带 `std::vector<int>` 的接口

```c++
template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   const std::vector<int> &indices,
                   Eigen::Matrix<Scalar, 4, 1> &centroid);

template <typename PointT, typename Scalar> inline unsigned int
compute3DCentroid (const pcl::PointCloud<PointT> &cloud,
                   const pcl::PointIndices &indices,
                   Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  return (pcl::compute3DCentroid (cloud, indices.indices, centroid));
}
```

