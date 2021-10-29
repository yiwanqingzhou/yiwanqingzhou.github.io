---
title: C++并发编程：std::future的使用
date: 2021-10-29 16:39:55
tags:
- c++
- async
- 并发编程
categories:
- c++
keywords:
description:
---



`std::future` 是C++11 的一个模板类，**提供了一种用于访问异步操作结果的机制**。可以用来获取异步任务的结果，因此可以把它当成一种简单的线程间同步的手段。



设想这样的情况，你希望一个线程进行工作A，同时你在做一些其他的工作，你希望在某个特定的时间获取那个工作A的结果。在c++11，这个可以轻松被 `std::future` 实现。而由于它是一个模板类，可以返回任何类型的结果。



<!-- more -->



```C++
std::future<Result> fut = std::async(std::launch::async, []() {return A();});

// ...
// 进行别的工作

// 获取A的结果 
Result result = fut.get();
```



#### 用法说明

`std::future` 对象通常由某个 `Provider` 创建，你可以把 `Provider` 想象成一个异步任务的提供者，`Provider` 在某个线程中设置共享状态的值，与该共享状态相关联的 `std::future` 对象（通常在另外一个线程中）调用 `std::future::get()` 获取该值。如果共享状态的标志不为 `std::future_status::ready`，则调用 `get()` 会**阻塞**当前的调用者，直到 `Provider` 设置了共享状态的值，`get()` 返回异步任务的返回值或发生的异常。



#### 创建

`std::future` 的拷贝构造函数和普通赋值操作是被禁用的，只提供了默认的构造函数和 `move` 构造函数。默认构造函数构造的对象没有共享状态，因此它是无效的，但是可以通过移动赋值的方式将一个有效的`future` 值赋值给它。



一个有效的 `std::future` 对象通常由以下三种 `Provider` 创建，并和某个共享状态相关联。

- `std::async()` 函数
- `std::promise::get_future()`，为 `promise` 类的成员函数
- `std::packaged_task::get_future()`，为 `packaged_task` 的成员函数

```c++
bool test(int d);

std::future<int> fut;           // 默认构造函数
fut = std::async(std::async(test, 1));   // move-赋值操作。
```



#### 成员函数

- `bool valid()`

  检查共享状态的有效性，返回当前的 `future` 对象是否与共享状态关联。一旦调用了 `std::future::get()` 函数，再调用此函数将返回 `false` 。

- `void wait()`

  - 等待共享状态就绪
  - 如果共享状态尚未就绪(即未返回或发生异常)，则该函数将阻塞调用的线程直到就绪
  - 当共享状态就绪后，则该函数将取消阻塞并void返回

- `std::future_status wait_for(std::chrono::duration span)`

  - 在指定的时间内等待共享状态就绪
  - 如果共享状态尚未就绪，则该函数将阻塞调用的线程直到就绪或已达到设置的时间
  - 返回`std::future_status`：`ready` / `timeout` / `deferred`

  ```C++
  void test(int d)
  {
    std::this_thread::sleep_for(std::chrono::seconds(d));
  }
  
  int main()
  {
      std::future<bool> fut = std::move(std::async(test, 1));
      
      auto start = std::chrono::steady_clock::now();
      auto future_status = fut.wait_for(std::chrono::seconds(2));
      auto end = std::chrono::steady_clock::now();
      std::chrono::duration<double> elapsed_seconds = end - start;
      std::cout << "waited for : " << elapsed_seconds.count() << "s\n";
      
      auto result = fut.get();
  }
  ```

  输出：

  ```
  waited for : 1.00856s
  ```

  

- `std::future_status wait_until(std::chrono::time_point point)`

  - 在指定的时间点前等待共享状态准备就绪
  - 如果共享状态尚未就绪，则该函数将阻塞调用的线程直到就绪或已达到指定的时间点
  - 返回`std::future_status`：`ready` / `timeout` / `deferred`

- `_Res get()`

  - 当共享状态就绪时，返回存储在共享状态中的值(或抛出异常)
  - 如果共享状态尚未就绪，则该函数将阻塞调用的线程直到就绪，相当于 `wait()` 再 `get()`
  - 当共享状态就绪后，则该函数将取消阻塞并返回释放其共享状态，这使得 `future` 对象不再有效，因此对于每一个 `future` 共享状态，`get()`函数最多应被调用一次
  - `std::future<void>::get()` 不返回任何值，但仍等待共享状态就绪并释放它

- ` std::shared_future<_Res> share()`
  - 返回一个 `std::shared_future` 对象，该对象获取 `future` 对象的共享状态。`future` 对象将不再有效。





#### 简单例子

```c++
#include <future>
#include <chrono>
#include <iostream>

bool test(int d)
{
  std::cout << "test()\n";
  std::this_thread::sleep_for(std::chrono::seconds(5));
  std::cout << "5 sec later..\n";
  return true;
}

int main(int argc, char **argv)
{
  std::future<bool> fut = std::move(std::async(test, 1));

  std::cout << "waiting..\n";

  // -1-
  auto timeout = std::chrono::seconds(2);
  std::future_status future_status = fut.wait_for(timeout);
  if (future_status != std::future_status::ready)
  {
    std::cout << "2 sec timeout!\n";
    // return -1;
  }

  // -2-
  fut.wait();

  bool result = fut.get();
  std::cout << "\nresult : " << result << std::endl;
  return 0;
}

```





#### `std::shared_future`

`shared_future` 与 `future` 类似，但是允许多个线程等待同一个共享状态。 `shared_future` 既支持移动操作也支持拷贝操作，而且多个 `shared_future` 对象可以引用相同的共享状态，还允许多次检索共享状态下的值（多次调用 `get()` ）。

`shared_future` 可以通过某个 `future` 对象隐式转换，或者通过 `std::future::share()` 显示转换，无论哪种转换，被转换的那个 `std::future` 对象都会变为 `not-valid`.



#### **std::launch**

该枚举类型主要是在调用` std::async` 设置异步任务的启动策略的。

`std::async`的原型:

```c++
std::async(std::launch::async | std::launch::deferred, f, args...)
```



- `std::launch::async`

   表示在调用`async`函数的时候就开始创建新线程。

-  `std::launch::deferred`

  表示延迟调用，在调用 `future` 中的 `wait()` 或者 `get()` 函数时，才执行入口函数。（实际上，并没有创建新线程，只是在主线程中调用的入口函数）

```C++
// async
auto fut = std::async([]() {return A();});
auto fut = std::async(std::launch::async, []() {return A();});

// deferred
auto fut = std::async(std::launch::deferred, []() {return A();});
```

