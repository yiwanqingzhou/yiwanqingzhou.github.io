---
title: C++ 类构造函数
date: 2022-01-29 11:34:32
tags:
- c++
- 构造
categories:
- c++
keywords:
- constructor
description:
---



# 构造函数

类的**构造函数**是类的一种特殊的成员函数，用于创建类的新对象和初始化自定义类成员。

- 构造函数具有与类相同的名称，没有返回值，也不返回 `void`。
- 可以根据需要定义多个重载构造函数，以各种方式自定义初始化。
- 构造函数可用于为某些成员变量设置初始值。
- 通常构造函数具有**公共**可访问性，因此外部代码可以调用其创建类的对象，但也可以将构造函数声明为 **protected** 或 **private** 。
- 构造函数可以声明为 **`inline`, `explicit`, `friend` 或 `constexpr`**
- 构造函数可以初始化已声明为 `const`, `volatile` 或者`const volatile ` 的对象，该对象在构造完成后变为 `const`
- 如果没有自己声明，编译器将为一个类声明 (编译器版本的) **默认构造函数、复制构造函数、复制赋值操作符和析构函数**。所有这些函数都是 `public` 且 `inline` 的。



## 成员初始化列表 Member Initializer List

构造函数可以有一个成员初始化列表，用于在执行构造函数体之前初始化类成员。

使用成员初始化列表比在构造函数体中赋值的效率更高，因为它直接初始化成员变量，而不需要调用赋值构造。

**对基类构造函数的调用应在初始化器列表中进行，以确保基类在执行派生构造函数之前已完全初始化。**

```c++
class Base
{
  int b_;

public:
  Base(int b) : b_(b) { std::cout << "base constructor" << std::endl; }
};

class Derived : public Base
{
  int d_;

public:
  Derived(int b, int d) : Base(b), d_(d) { std::cout << "derived constructor" << std::endl; }
};
```

可以指定无物 (nothing) 作为初始化实参：

```c++
Derived() : Base(), d_(0) {};
```





## 默认构造 Default Constructor

默认构造函数通常没有参数，但它们可以具有具有默认值的参数。调用默认构造函数时，不应使用括号 (会被视为函数声明) :

```c++
class Box {
public:
    // 没有参数
    Box() { /*perform any required default initialization steps*/}

    // 所有参数都具有默认值
    Box (int w = 1, int l = 1, int h = 1): m_width(w), m_height(h), m_length(l){}
private:
    int m_width, m_length, m_height;
}

int main()
{
    Box box1;
    Box box2(1, 2, 3);
    Box box3(); // warning C4930: prototyped function not called (was a variable definition intended?)
}
```



默认构造函数是 **特殊成员函数** 之一，如果未在类中声明任何构造函数，编译器将提供隐式默认的 **inline** 构造函数。如果依赖于隐式默认构造函数，需要确保初始化所有成员变量。

```c++
#include <iostream>
using namespace std;

class Box {
public:
    int Volume() {return m_width * m_height * m_length;}
private:
    int m_width{0}; // 初始化成员变量
    int m_height{0};
    int m_length{0};
};

int main() {
    Box box1; // 调用编译器自动生成的默认构造函数
    cout << "box1.Volume: " << box1.Volume() << endl; // Outputs 0
}
```



如果声明了任何非默认构造函数，则编译器不提供默认构造函数。没有默认构造函数时，该类的对象数组不能只使用方括号语法构造。

```c++
class Box {
public:
    Box(int width, int length, int height)
        : m_width(width), m_length(length), m_height(height){}
private:
    int m_width, m_length, m_height;
};

int main(){
    Box box; // C2512: no appropriate default constructor available
    Box boxes[3]; // C2512: no appropriate default constructor available
    Box boxes1[3]{ { 1, 2, 3 }, { 4, 5, 6 }, { 7, 8, 9 } };
}
```





## 复制构造 Copy Constructor

复制构造函数通过从相同类型的对象中复制成员值来初始化对象。

如果类成员都是标准简单类型，编译器自动生成的复制构造函数就足够了。但如果成员变量存在指针等复杂类型，那么需要自定义复制构造函数，用于分配新的内存等操作。

**Pass-by-value 将自动调用复制构造函数。**



```c++
Box(Box& other); // Avoid if possible--allows modification of other.
Box(const Box& other);
Box(volatile Box& other);
Box(volatile const Box& other);

// Additional parameters OK if they have default values
Box(Box& other, int i = 42, string label = "Box");
```



通过将复制构造函数定义为已删除，可以防止对象被复制

```c++
Box (const Box& other) = delete;
```



**如果定义了复制构造函数，还应定义复制赋值运算符 (=)。**

 

## 赋值运算符 Copy Assignment

赋值运算符 (**=**) 严格地说是二元运算符，但比较特殊：

- 必须是**非静态** **成员函数**。 
- 不会被派生类继承。
- 如果不手动定义，编译器会自动生成一个默认的赋值运算符

```c++
class Box
{
public:
    // Right side of copy assignment is the argument.
    Box& operator=(const Box&);
private:
    int m_width, m_length, m_height;
};

// Define copy assignment operator.
Box& Box::operator=(const Box& other)
{
    if (this != &other)
    {
        m_width = other.m_width;
        m_length = other.m_length;
        m_height = other.m_height;
    }

    // Assignment operator returns left side of assignment.
    return *this;
}
```



**复制构造函数和赋值运算符的区别**：

```c++
Box box1;
Box box2 = box1; // Copy constructor is called
Box box3(box1); // Copy constructor call.
```





## 移动构造 Move Constructor

移动构造函数是一种特殊成员函数，它无需复制原始数据即可将现有对象数据的所有权移动到新对象。它采用左值引用作为第一个参数，任何其他参数都必须具有默认值。移动构造函数可以在传递大型对象时显著提高程序的效率。

```c++
Box(Box&& other);
```



如果对象由即将销毁且不再需要其资源的另一个相同类型的对象初始化，编译器会选择使用移动构造函数。

```c++
class TestClass
{
  int a_;
  std::string b_;

public:
  TestClass(const int &a, const std::string &b) : a_(a), b_(b)
  {
    std::cout << "default" << std::endl;
  };
  TestClass(const TestClass &other) : a_(other.a_), b_(other.b_)
  {
    std::cout << "copy" << std::endl;
  }
  TestClass(TestClass &&other) : a_(other.a_), b_(std::move(other.b_))
  {
    std::cout << "move" << std::endl;
  }
};

int main()
{
  std::vector<TestClass> vec;
  vec.reserve(1);
  vec.push_back(TestClass(1, "3"));
  return 0;
}
```

输出：

```bash
default
move
```

如果**没有定义移动构造函数**，则输出为：

```bash
default
copy
```



- 如果没有定义移动构造函数，且没有用户声明的复制构造函数、复制赋值运算符、移动赋值运算符或析构函数，编译器将生成隐式构造函数。

- 如果未定义显式或隐式移动构造函数，则使用移动构造函数的操作将被改为使用复制构造函数。

- 如果定义了移动构造函数或移动赋值运算符，则隐式声明的复制构造函数将被定义为 `deleted`。

- 如果作为类类型的任何成员缺少析构函数或编译器无法确定要用于移动操作哪个构造函数，则隐式声明移动构造函数定义为 `deleted`。

  


## 委托构造函数 Delegating Constructor

委托构造函数调用同一类中的不同构造函数来执行某些初始化工作。

可以在一个构造函数中编写主逻辑，并从其他构造函数调用它。

```c++
class Box {
public:
    Box(int i) :  Box(i, i, i) {}  // 委托构造
    Box(int width, int length, int height)
        : m_width(width), m_length(length), m_height(height) {}
};
```




## explicit 显式构造

将构造函数声明为 `explicit` 可以防止隐式类型转换

```c++
class Test
{
public:
    Test() { cout << "Test()" << endl; }
    explicit Test(int i) : num(i) { cout << "Test(int)" << endl; }
    explicit Test(char c) : letter(c) { cout << "Test(char)" << endl; }

private:
    int num;
    char letter;
};

int main()
{
    Test t1(1);
    Test t2('2');
    Test t = 3; // 编译器错误
}
```





## 显式默认和删除

- 可以显式默认复制构造函数、默认构造函数、move构造函数、复制赋值操作符、move赋值操作符和析构函数。
- 可以显式地删除所有特殊成员函数。

```c++
class Noncopyable
{
public:
    Noncopyable() = default;
    Noncopyable(const Noncopyable& other) = delete;
    Noncopyable& operator=(const Noncopyable& other) = delete;
    Noncopyable(Noncopyable&& other) = delete;
    Noncopyable& operator=(Noncopyable&& other) = delete;
    //...
};
```



```c++
class DoubleOnly
{
  double d_;

public:
  DoubleOnly(float) = delete;
  DoubleOnly(double d) : d_(d) {}
};
```



一些规则：

- 如果显式声明了任何构造函数，则不会自动生成默认构造函数。
- 如果显式声明了虚拟析构函数，则不会自动生成默认析构函数。
- 如果显式声明了移动构造函数或移动赋值运算符，则：
  - 不自动生成复制构造函数。
  - 不自动生成复制赋值运算符。
- 如果显式声明了复制构造函数、复制赋值运算符、移动构造函数、移动赋值运算符或析构函数，则：
  - 不自动生成移动构造函数。
  - 不自动生成移动赋值运算符。





## 构造顺序

构造函数按此顺序执行工作：

- 按声明顺序调用基类和成员构造函数。

- 如果类继承自一个虚拟基类，将对象的虚拟基指针初始化。

- 如果类具有或继承了虚函数，将对象的虚函数指针初始化。虚函数指针指向类中的虚函数表，确保虚函数正确地调用绑定代码。

- 执行构造函数体中的代码。



```c++
class Contained1
{
public:
  Contained1() { cout << "Contained1 ctor\n"; }
};

class Contained2
{
public:
  Contained2() { std::cout << "Contained2 ctor\n"; }
};

class Contained3
{
public:
  Contained3() { std::cout << "Contained3 ctor\n"; }
};

class BaseContainer
{
public:
  BaseContainer() { std::cout << "BaseContainer ctor\n"; }

private:
  Contained1 c1;
  Contained2 c2;
};

class DerivedContainer : public BaseContainer
{
public:
  DerivedContainer() : BaseContainer() { std::cout << "DerivedContainer ctor\n"; }

private:
  Contained3 c3;
};

int main()
{
  DerivedContainer dc;
  return 0;
}
```

输出：

```bash
Contained1 ctor
Contained2 ctor
BaseContainer ctor
Contained3 ctor
DerivedContainer ctor
```





参考：https://docs.microsoft.com/zh-cn/cpp/cpp/constructors-cpp?view=msvc-170