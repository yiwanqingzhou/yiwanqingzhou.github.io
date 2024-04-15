---
title: C++ push_back和emplace_back的区别
date: 2022-01-28 17:26:02
tags:
- c++
- stl
- vector
categories:
- c++
keywords:
- c++
- stl
- vector
description:
---



> `push_back() `向容器尾部添加元素时，首先会创建这个元素，然后再将这个元素拷贝或者移动到容器中(如果是拷贝的话，事后会自行销毁先前创建的这个元素)。
>
> 而 `emplace_back()` 在实现时，则是直接在容器尾部创建这个元素，省去了拷贝或移动元素的过程。



事实上，大多数情况下二者没有区别，只有在少数情况下 `emplace_back()` 效率更高。主要是 `emplace_back()` 支持 `in-place construction` 。



直接看下代码：



```c++
class TestClass
{
  int a_;
  std::string b_;

public:
  TestClass(const int &a, const std::string &b) : a_(a), b_(b)
  {
    std::cout << "default constructor" << std::endl;
  };
  TestClass(const TestClass &other) : a_(other.a_), b_(other.b_)
  {
    std::cout << "copy constructor" << std::endl;
  }
  TestClass(TestClass &&other) : a_(other.a_), b_(std::move(other.b_))
  {
    std::cout << "move constructor" << std::endl;
  }
  ~TestClass() { std::cout << "destructor" << std::endl; }
};

int main(int argc, char **argv)
{
  std::vector<TestClass> vec;
  vec.reserve(100);

  TestClass t1(1, "1");
  TestClass t2(2, "2");

  std::cout << "\n-----------------0---------------\n";

  vec.push_back(t1);
  vec.clear();
  std::cout << "\n";
  vec.emplace_back(t1);
  vec.clear();
  std::cout << "-----------------1---------------\n";

  vec.push_back(std::move(t1));
  vec.clear();
  std::cout << "\n";
  vec.push_back(std::move(t2));
  vec.clear();
  std::cout << "----------------2----------------\n";

  vec.push_back(TestClass(3, "3"));
  vec.clear();
  std::cout << "\n";
  vec.emplace_back(TestClass(3, "3"));
  vec.clear();
  std::cout << "----------------3----------------\n";

  vec.push_back({4, "4"});
  vec.clear();
  std::cout << "\n";
  vec.emplace_back(4, "4"); // in-place construction
  vec.clear();
  std::cout << "----------------4----------------\n";

  return 0;
}
```



输出：

```bash
default constructor
default constructor

-----------------0---------------
copy constructor
destructor

copy constructor
destructor
-----------------1---------------
move constructor
destructor

move constructor
destructor
----------------2----------------
default constructor
move constructor
destructor
destructor

default constructor
move constructor
destructor
destructor
----------------3----------------
default constructor
move constructor
destructor
destructor

default constructor
destructor
----------------4----------------

destructor
destructor

```



从结果来看，只有直接调用构造有区别

```c++
vec.push_back({4, "4"});
vec.emplace_back(4, "4");
```

在这种情况下，`push_back()` 会调用默认构造和移动构造，对应两次析构；而 `emplace_back()` 只调用一次默认构造一次析构， 效率更高。
