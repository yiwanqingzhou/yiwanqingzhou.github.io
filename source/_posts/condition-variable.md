---
title: C++并发编程：条件变量std::condition_variable
date: 2021-10-27 16:43:24
tags:
- c++
- async
- 并发编程
categories:
- note
keywords:
description:
---



`std::condition_variable` 是 C++11 多线程编程中的条件变量。

一般用法: 线程 A **等待**某个条件并挂起，直到线程 B 设置了这个条件，并**通知**条件变量，然后线程 A 被唤醒。



<!-- more -->



#### 构造

仅支持默认构造函数，拷贝、赋值和移动(move)均是被禁用的。

```c++
std::condition_variable cv;
```





#### 等待

##### 无条件等待

阻塞当前线程直到被其他线程通知唤醒。

```c++
void wait (unique_lock<mutex>& lck);
```

##### 有条件等待

只有当 `pred` 条件为 `false` 时才会阻塞当前线程，并且在收到其他线程的通知后只有当 `pred` 为 `true` 时才会被解除阻塞。

```c++
template <class Predicate>
void wait (unique_lock<mutex>& lck, Predicate pred);
```

相当于

```c++
while (!pred())
{
	wait()
}
```





#### 通知

通知线程可以使用 `notify_one()` 通知一个线程，或一次使用 `notify_all()` 通知所有线程。





#### 简单例子

```c++
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>


std::mutex mtx;
std::condition_variable cv;
std::atomic<bool> ready;

void wait(int id)
{
  std::cout << "thread: " << std::this_thread::get_id() << " id: " << id << " start.\n";

  std::unique_lock<std::mutex> lck(mtx);
  cv.wait(lck, [] { return ready.load(); });

  std::cout << "thread: " << std::this_thread::get_id() << " id: " << id << " done.\n";
}

void set_ready()
{
  std::cout << "set_ready(): " << std::endl;
  ready.store(true);
  cv.notify_one();
}

int main(int argc, char **argv)
{
  ready.store(false);

  auto t_1 = std::thread(wait, 1);
  auto t_2 = std::thread(wait, 2);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  set_ready();

  std::this_thread::sleep_for(std::chrono::seconds(1));
  set_ready();

  std::cout << "join...." << std::endl;
  t_1.join();
  t_2.join();
  std::cout << "all done" << std::endl;

  return 0;
}

```



输出:

```
thread: 139827702187776 id: 1 start.
thread: 139827693795072 id: 2 start.
set_ready(): 1
thread: 139827702187776 id: 1 done.
set_ready(): 1
join....
thread: 139827693795072 id: 2 done.
all done
```



