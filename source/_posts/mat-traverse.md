---
title: OpenCV Mat像素的遍历方法
date: 2022-03-28 16:27:54
tags:
- c++
- opencv
categories:
- cv
keywords:
description:
---

## OpenCV像素遍历常用的几种方法

以从 organized 的点云提取 RGB 信息为例



### 动态地址at

基于Mat对象的随机像素访问 API 实现，通过行列索引方式遍历每个像素值。这种方法速度较慢，不太适合用于像素遍历。

```C++
void extract_1(const pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &image)
{
  auto &cloud_height = cloud->height;
  auto &cloud_width = cloud->width;

  image = cv::Mat(cv::Size(cloud_width, cloud_height), CV_8UC3);

#pragma omp parallel for
  for (size_t row = 0; row < cloud_height; row++)
  {
    for (size_t col = 0; col < cloud_width; col++)
    {
      auto index = row * cloud_width + col;
      const auto &pt = cloud->points[index];

      image.at<cv::Vec3b>(row, col)[0] = pt.b;
      image.at<cv::Vec3b>(row, col)[1] = pt.g;
      image.at<cv::Vec3b>(row, col)[2] = pt.r;
    }
  }
}
```



### 行指针

基于Mat对象的行随机访问指针方式实现对每个像素的遍历。

```C++
void extract_2(const pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &image)
{
  auto &cloud_height = cloud->height;
  auto &cloud_width = cloud->width;

  image = cv::Mat(cv::Size(cloud_width, cloud_height), CV_8UC3);

#pragma omp parallel for
  for (size_t row = 0; row < cloud_height; row++)
  {
    cv::Vec3b *ptr = image.ptr<cv::Vec3b>(row);
    for (size_t col = 0; col < cloud_width; col++)
    {
      auto index = row * cloud_width + col;
      const auto &pt = cloud->points[index];

      cv::Vec3b &pixel = ptr[col];
      pixel[0] = pt.b;
      pixel[1] = pt.g;
      pixel[2] = pt.r;
    }
  }
}
```



### uchar 行指针 

```C++
void extract_3(const pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &image)
{
  auto &cloud_height = cloud->height;
  auto &cloud_width = cloud->width;

  image = cv::Mat(cv::Size(cloud_width, cloud_height), CV_8UC3);

#pragma omp parallel for
  for (size_t row = 0; row < cloud_height; row++)
  {
    uchar *uc_pixel = image.ptr(row);
    for (size_t col = 0; col < cloud_width; col++)
    {
      auto index = row * cloud_width + col;
      const auto &pt = cloud->points[index];

      uc_pixel[0] = pt.b;
      uc_pixel[1] = pt.g;
      uc_pixel[2] = pt.r;
      uc_pixel += 3;
    }
  }
}
```



### 将图像完全展开

一般图像行与行之间往往存储是不连续的，但是有些图像可以是连续的，Mat提供了一个检测图像是否连续的函数`isContinuous()`。当图像连通时，我们就可以把图像完全展开，看成是一行进行处理。



### 速度比较

```c++
int main()
{
  std::string cloud_path = "test.pcd"
  pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  pcl_utils::load_pcd(cloud_path, *cloud);

  std::cout << *cloud << std::endl;

  auto start = std::chrono::steady_clock::now();
  cv::Mat img_1;
  extract_1(cloud, img_1);
  auto end = std::chrono::steady_clock::now();
  auto elapsed_seconds = static_cast<std::chrono::duration<double>>(end - start);
  std::cout << "1 elapsed time: " << elapsed_seconds.count() << "s\n";

  start = std::chrono::steady_clock::now();
  cv::Mat img_2;
  extract_2(cloud, img_2);
  end = std::chrono::steady_clock::now();
  elapsed_seconds = static_cast<std::chrono::duration<double>>(end - start);
  std::cout << "2 elapsed time: " << elapsed_seconds.count() << "s\n";

  start = std::chrono::steady_clock::now();
  cv::Mat img_3;
  extract_3(cloud, img_3);
  end = std::chrono::steady_clock::now();
  elapsed_seconds = static_cast<std::chrono::duration<double>>(end - start);
  std::cout << "3 elapsed time: " << elapsed_seconds.count() << "s\n";

  return 0;
}
```



输出 :

```bash
Load pcd file with 1228800 data points.
header: seq: 0 stamp: 0 frame_id: 

points[]: 1228800
width: 1280
height: 960
is_dense: 0
sensor origin (xyz): [0, 0, 0] / orientation (xyzw): [0, 0, 0, 1]

1 elapsed time: 0.0199077s
2 elapsed time: 0.012727s
3 elapsed time: 0.00880297s
4 elapsed time: 0.00900436s
```

