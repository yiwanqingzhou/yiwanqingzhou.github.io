---
title: C++ std::vector的内存分配机制
date: 2022-01-05 16:02:21
tags:
- c++
- stl
- vector
categories:
- c++
keywords:
- 内存分配
- C++
- vector
- stl
- 顺序容器
description:
---



> `template < class T, class Alloc = allocator<T> > class vector; `



### std::vector 简介

`std::vector` 是C++标准库里封装好的**动态大小数组**的顺序容器，能够存放各种类型的对象。

与数组 `array` 一样， `vector` 的**内存空间的地址是连续的**。这意味着可以通过下标索引的方式获取到对应的元素，所以访问其元素的效率非常高，从其末端添加或删除元素的效率也相对较高。而对于涉及在非结束位置插入或删除元素的操作，它们的性能比其他操作差，效率较低。

但与`array`不同的是，它们的大小可以动态变化，它们的存储由容器自动处理。在插入新元素时，若当前容量不能够容纳新的元素，将自动重新申请一块更大的内存空间，将原有数据拷贝到新的内存空间，且释放原来的空间。这一过程非常耗时，为了避免频繁的内容分配， `vector` 不会在每次添加元素时都重新分配空间，而是分配一些额外的存储空间来容纳可能的增长。因此， `vector` 的实际容量 (**capacity**) 永远大于等于它容纳的元素大小 (**size**)。



### 内存分配机制



#### 1. 自动增长策略

假设元素是连续存储的，并且容器的大小是可变的，如果此时向 vector 中添加新的元素，容器不可能简单地将它添加到内存的其它位置，因为元素必须是连续存储的。

容器必须分配新的空间，来保存已有元素和新的元素，将已有的元素从旧位置移动到新空间。然后添加新元素，释放旧的存储空间。如果每添加一个元素，容器就执行一次内存分配和释放，性能会变得超级慢。

为了避免这种代价，标准库实现者采用了可以减少容器空间重新分配的策略。当不得不获取新的空间的时候，`vector` 的实现，通常会分配比需求空间更大的内存空间。这种分配策略，比每次添加新元素后都重新分配容器内存空间的策略要高效的多。



```c++
int main(int argc, char **argv)
{
  std::vector<int> vec;
  std::cout << "1 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.push_back(1);
  std::cout << "2 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.push_back(2);
  std::cout << "3 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.push_back(3);
  std::cout << "4 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.push_back(4);
  std::cout << "5 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.push_back(5);
  std::cout << "6 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  return 0;
}
```



输出：

```bash
1 vec capacity: 0  size: 0
2 vec capacity: 1  size: 1
3 vec capacity: 2  size: 2
4 vec capacity: 4  size: 3
5 vec capacity: 4  size: 4
6 vec capacity: 8  size: 5
```



可以看出，每当 `size` 和 `capacity` 相等时，也就是无法容纳新的元素时，`vector` 自动申请了新的 (成倍增长的) 容量。



#### 2. 手动分配内存: reserve 和 resize

`std::vector ` 有自动分配内存的机制，但我们也可以通过`reserve()` 和 `resize()` 来手动分配内存，使其效率更高。



```c++
void print_ele(const std::vector<int> &vec)
{
  std::cout << "ele: ";
  for (auto &e : vec)
  {
    std::cout << e << " ";
  }
  std::cout << std::endl;
}

int main()
{
  std::vector<int> vec;

  vec.reserve(4);
  std::cout << "1 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.push_back(1);
  vec.push_back(2);
  vec.push_back(3);
  vec.push_back(4);
  std::cout << "2 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.reserve(3);
  std::cout << "3 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;
  print_ele(vec);

  vec.resize(5);
  std::cout << "4 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;
  print_ele(vec);

  vec.resize(3);
  std::cout << "5 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;
  print_ele(vec);

  vec.resize(5);
  std::cout << "6 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;
  print_ele(vec);

  return 0;
}
```



输出：

```bash
1 vec capacity: 4  size: 0
2 vec capacity: 4  size: 4
3 vec capacity: 4  size: 4
ele: 1 2 3 4 
4 vec capacity: 8  size: 5
ele: 1 2 3 4 0 
5 vec capacity: 8  size: 3
ele: 1 2 3 
6 vec capacity: 8  size: 5
ele: 1 2 3 0 0 
```



可以看出：

1.  `reserve()` 只增加不减少数组的 `capacity`，不对 `size()` 造成任何改变
2.  `resize()` 只增加不减少数组的 `capacity`，但可以增加和减少 `size`。减少时会直接移除多余的元素，增加时会填入默认值 (0)。



#### 3. 手动回收内存

##### erase

`erase()` 可以从 `vector` 中移除单个或一段元素 [begin, end)，实际上是以后面的元素移动并覆盖前面的位置，不对`capacity` 造成改变。

```c++
int main()
{
  std::vector<int> vec{1, 2, 3, 4, 5};
  std::cout << "1 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.erase(vec.begin() + 1);
  std::cout << "2 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;
  print_ele(vec);

  vec.erase(vec.begin(), vec.begin() + 2);
  std::cout << "3 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;
  print_ele(vec);

  return 0;
}
```

输出：

```bash
1 vec capacity: 5  size: 5
2 vec capacity: 5  size: 4
ele: 1 3 4 5 
3 vec capacity: 5  size: 2
ele: 4 5 
```



##### clear

`clear()` 可以移除 `vector`所有元素，使容器`size` 为0，不对`capacity` 造成改变。

```C++
int main()
{
  std::vector<int> vec{1,2,3,4};
  std::cout << "1 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.clear();
  std::cout << "2 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  return 0;
}
```

输出：

```bash
1 vec capacity: 4  size: 4
2 vec capacity: 4  size: 0
```



##### shrink_to_fit (c++11)

`shrink_to_fit()` 可以请求将内存减少到等于当前元素实际所使用的大小，也就是使 `capacity = size`

```c++
int main()
{
  std::vector<int> vec(10);
  std::cout << "1 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.resize(1);
  std::cout << "2 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  vec.shrink_to_fit();
  std::cout << "3 vec capacity: " << vec.capacity() << "  size: " << vec.size() << std::endl;

  return 0;
}
```



输出：

```bash
1 vec capacity: 10  size: 10
2 vec capacity: 10  size: 1
3 vec capacity: 1  size: 1
```

