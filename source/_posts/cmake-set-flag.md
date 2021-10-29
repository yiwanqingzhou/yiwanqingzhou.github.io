---
title: CMake设置编译选项
date: 2021-10-14 10:03:57
tags: 
- cmake
- 编译
categories:
- cmake
encrypt:
description:
---



#### CMake中设置编译选项

```cmake
option(BUILD_VISUALIZATIONS "Build visualization, default OFF" OFF)
message("BUILD_VISUALIZATIONS is ${BUILD_VISUALIZATIONS}")


# 可以跟build type关联
# if(CMAKE_BUILD_TYPE MATCHES Debug OR CMAKE_BUILD_TYPE MATCHES DEBUG)

if(BUILD_VISUALIZATIONS)
add_definitions(-DBUILD_VISUALIZATIONS)
# target_compile_definitions(${project_name} PRIVATE BUILD_VISUALIZATIONS)
endif()
```



#### 代码中作为宏

```C++
#ifdef BUILD_VISUALIZATIONS

void visualize_cloud(const cloud_ptr_t& cloud_ptr)
{
std::cout << "visualizations on" << std::endl;
// visualize cloud
}

#else

template <typename... Args>
void visualize_cloud(Args...) {
std::cout << "pass visualizations" << std::endl;
}

#endif
```



#### 编译时传入参数

```bash
--cmake-args -DBUILD_VISUALIZATIONS=ON
```

