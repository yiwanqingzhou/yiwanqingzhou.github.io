---
title: 使用PCL的滤波时遇到`double free or corruption`
date: 2024-04-08 10:35:32
tags:
- pcl
- eigen
- cmake
- c++
- 编译
categories:
- [pcl]
- [c++]
- [cmake]
keywords:
- pcl
- eigen
- c++
- cmake
description:
---

在调用 `StatisticalOutlierRemoval` 时遇到 `double free or corruption`

我在 `PCL` 源代码中加入了一些log，具体如下：

```bash
2: [INFO]1712543988.313661216: filter - CropBox
2: --- FilterIndices h - filter indices - start ---
2: --- PCLBase - initCompute - indices size:1228800 ---
2: --- FilterIndices h - call applyFilter(indices)
2: ----- CropBox hpp - applyFilter indices - start -------
2: ----- CropBox hpp - applyFilter indices - finish -------
2: --- FilterIndices h - call deinitCompute()
2: --- PCLBase hpp - deinitCompute ---
2: --- FilterIndices h - filter indices - finish ---

2: [INFO]1712543988.372619178: filter - StatisticalOutlierRemoval
2: --- FilterIndices h - filter indices - start ---
2: --- PCLBase - initCompute - indices size:2568 ---
2: --- FilterIndices h - call applyFilter(indices)
2: --- StatisticalOutlierRemoval h - applyFilter indices ---
2: ----- StatisticalOutlierRemoval hpp - applyFilterIndices - start -------
2: ----- StatisticalOutlierRemoval hpp - applyFilterIndices - finish -------
2: --- FilterIndices h - call deinitCompute()
2: double free or corruption (out)
```

[PCL 的 filter 函数里](https://github.com/PointCloudLibrary/pcl/blob/master/filters/include/pcl/filters/filter_indices.h#L99-L109)只包括了 `initCompute()` `applyFilter()` `deinitCompute()` 三个部分。

在正常情况，也就是跑 `CropBox` 的时候，程序会进入 `deinitCompute()`。而下面跑 `StatisticalOutlierRemoval` 的时候，明显它没有进入到 `deinitCompute()`，没有打印里面的log，就以 `double free or corruption (out)` 结束了。

### 使用 `gdb` 追踪

```bash
--- FilterIndices h - call deinitCompute()
double free or corruption (out)

Thread 1 "tests" received signal SIGABRT, Aborted.
__pthread_kill_implementation (no_tid=0, signo=6, threadid=140736700039680) at ./nptl/pthread_kill.c:44
44	./nptl/pthread_kill.c: No such file or directory.
(gdb) bt
#0  __pthread_kill_implementation (no_tid=0, signo=6, threadid=140736700039680) at ./nptl/pthread_kill.c:44
#1  __pthread_kill_internal (signo=6, threadid=140736700039680) at ./nptl/pthread_kill.c:78
#2  __GI___pthread_kill (threadid=140736700039680, signo=signo@entry=6) at ./nptl/pthread_kill.c:89
#3  0x00007fffe5311476 in __GI_raise (sig=sig@entry=6) at ../sysdeps/posix/raise.c:26
#4  0x00007fffe52f77f3 in __GI_abort () at ./stdlib/abort.c:79
#5  0x00007fffe5358676 in __libc_message (action=action@entry=do_abort, fmt=fmt@entry=0x7fffe54aab77 "%s\n")
    at ../sysdeps/posix/libc_fatal.c:155
#6  0x00007fffe536fcfc in malloc_printerr (str=str@entry=0x7fffe54ad790 "double free or corruption (out)") at ./malloc/malloc.c:5664
#7  0x00007fffe5371e70 in _int_free (av=0x7fffe54e9c80 <main_arena>, p=0x555555b18b10, have_lock=<optimized out>)
    at ./malloc/malloc.c:4588
#8  0x00007fffe5374453 in __GI___libc_free (mem=<optimized out>) at ./malloc/malloc.c:3391
#9  0x000055555556c6ba in std::_Sp_counted_base<(__gnu_cxx::_Lock_policy)2>::_M_release() ()
#10 0x000055555556d33e in void vision::cloud::filter_sor<pcl::PointXYZRGB>(pcl::PointCloud<pcl::PointXYZRGB>::Ptr const&, std::vector<int, std::allocator<int> >&, int const&, double const&, std::vector<int, std::allocator<int> > const&) ()
#13 0x0000555555568caa in CoreHelpers_Test::TestBody() ()
#14 0x00005555555a7bef in void testing::internal::HandleExceptionsInMethodIfSupported<testing::Test, void>(testing::Test*, void (testing::Test::*)(), char const*) ()
#15 0x000055555559bde6 in testing::Test::Run() ()
#16 0x000055555559bf65 in testing::TestInfo::Run() ()
#17 0x000055555559c519 in testing::TestSuite::Run() ()
#18 0x000055555559cc1f in testing::internal::UnitTestImpl::RunAllTests() ()
#19 0x00005555555a81b7 in bool testing::internal::HandleExceptionsInMethodIfSupported<testing::internal::UnitTestImpl, bool>(testing::internal::UnitTestImpl*, bool (testing::internal::UnitTestImpl::*)(), char const*) ()
#20 0x000055555559c02c in testing::UnitTest::Run() ()
#21 0x00005555555684d4 in main ()
```

其中 `vision::cloud::filter_sor` 是我自定义的函数，里面就是调用了 `PCL` 的 `StatisticalOutlierRemoval`。

从 `gdb` 的结果来看，是 `PCL` 内部 `free` 的时候出了问题。

### Fix

> Eigen has a custom mechanism to guarantee aligned memory (used for everything older than C++17, see Memory.h in the Eigen project). If PCL is compiled with C++14 and the user project is compiled with C++17, this will lead to problems (e.g. memory allocated with the custom mechanism but freed without it). Defining EIGEN_HAS_CXX17_OVERALIGN=0 forces Eigen in the user project to use Eigen's custom mechanism, even in C++17 and newer.

也就是说，由于 `PCL` 使用了 `Eigen` 的一种特殊对齐方式，当 `PCL` 使用 `C++14` 但用户程序使用 `C++17` 编译时，由于内存对齐方式不一致，将可能导致 `double free`。

那么修复方式：
- 使用相同版本的 `C++` 编译`PCL`和用户程序
- 在 `PCL` 的 `cmake` 文件中[判断 `C++` 版本并设置`EIGEN_HAS_CXX17_OVERALIGN`](https://github.com/PointCloudLibrary/pcl/blob/master/cmake/pcl_pclconfig.cmake#L22-L24)
- 在用户程序的 `camke` 中设置 `EIGEN_HAS_CXX17_OVERALIGN`
  ```
  add_definitions(-DEIGEN_HAS_CXX17_OVERALIGN=0)
  ```
