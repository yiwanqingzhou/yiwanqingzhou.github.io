---
title: C++ 强制类型转换
date: 2022-01-24 18:38:46
tags:
- c++
categories:
- c++
keywords:
- static_cast
- dynamic_cast
- reinterpret_cast
- const_cast
description:
---





相比于C风格的强制类型转换，C++新增了关键字 `static_cast`、`dynamic_cast`、`const_cast`、`reinterpret_cast` ，用于强制类型转换。

新类型的强制转换可以提供更好的控制强制转换过程，允许控制各种不同种类的强制转换。



## 1. static_cast

用于非多态类型转换 (静态转换)，任何标准转换都可以用它，但是不能用于两个不相关的类型转换。


常用于：

  - 基本数据类型之间的转换，如`int -> char`，这种转换的安全也要开发人员来保证
  - 任何类型的表达式转换成 `void` 类型
  - 不能转换掉 `expression` 的 `const` 、`volitale` 或者 `__unaligned` 属性


```c++
int main()
{
  int a = 65;
  int b = 10;

  std::cout << "int a: " << a << std::endl;
  std::cout << "char a: " << static_cast<char>(a) << std::endl;

  double c = a / b;
  double d = static_cast<double>(a) / static_cast<double>(b);
  std::cout << "c: " << c << std::endl;
  std::cout << "d: " << d << std::endl;

  return 0;
}
```

输出：

```bash
a: 65
char a: A
c: 6
d: 6.5
```



**没有运行时类型检查来保证转换的安全性**

  - 上行转型 (子类对象指针-->父类对象指针/引用)： 安全
  - 下行转型(父类对象指针-->子类对象指针/引用)：没有动态类型检查，所以是不安全的

```c++
#include "typeinfo"

class A
{
public:
  int a = 1;
  virtual std::string class_name() final { return typeid(*this).name(); }
};

class B : public A
{
public:
  int b = 3;
};

class C : public A
{
public:
  int c = 5;
};
```


```c++
int main(int argc, char **argv)
{
  A *pt_1 = new A;
  A *pt_2 = new B;
  A *pt_3 = new C;

  auto b1 = static_cast<B *>(pt_1);
  auto b2 = static_cast<B *>(pt_2);
  auto b3 = static_cast<B *>(pt_3);

  std::cout << b1->class_name() << " " << b1->b << std::endl;
  std::cout << b2->class_name() << " " << b2->b << std::endl;
  std::cout << b3->class_name() << " " << b3->b << std::endl;

  return 0;
}
```

输出：

```bash
1A 32686
1B 3
1C 5
```



## 2. dynamic_cast

动态转换，常用于将一个父类对象的指针转换为子类对象的指针或引用。其他三种都是编译时完成的，dynamic_cast是运行时处理的，运行时要进行类型检查。

- 使用dynamic_cast进行转换的，基类中一定要有虚函数，否则编译不通过

- 在进行下行转换时，会进行类型检查 (这个信息存储在类的虚函数表)，比 `static_cast` 安全
- 转换后必须是类的指针、引用或 `void*`
- 对于指针，转换失败会返回`nullptr`；对于引用，转换失败会 (在运行时) 抛出异常



**返回指针**

```c++
int main(int argc, char **argv)
{
  A *pt1 = new A;
  A *pt2 = new B;
  A *pt3 = new C;

  auto b1 = dynamic_cast<B *>(pt1);
  auto b2 = dynamic_cast<B *>(pt2);
  auto b3 = dynamic_cast<B *>(pt3);

  if (b1)
    std::cout << b1->class_name() << " " << b1->b << std::endl;
  else
    std::cout << "b1 null" << std::endl;

  if (b2)
    std::cout << b2->class_name() << " " << b2->b << std::endl;
  else
    std::cout << "b2 null" << std::endl;

  if (b3)
    std::cout << b3->class_name() << " " << b3->b << std::endl;
  else
    std::cout << "b3 null" << std::endl;

  return 0;
}
```

输出：

```bash
b1 null
1B 3
b3 null
```



**返回引用**

```c++
int main()
{
  B b;
  C c;
  A &a1 = b;
  A &a2 = c;

  auto b1 = dynamic_cast<B &>(a1);
  std::cout << b1.class_name() << std::endl;

  auto b2 = dynamic_cast<B &>(a2);
  std::cout << b2.class_name() << std::endl;

  return 0;
}
```

输出：

```bash
1B
terminate called after throwing an instance of 'std::bad_cast'
  what():  std::bad_cast
Aborted
```



## std::dynamic_pointer_cast

使用 `std::dynamic_pointer_cast` 可以返回 `std::shared_ptr`

```C++
int main(int argc, char **argv)
{
  std::shared_ptr<A> pt1 = std::make_shared<B>();
  std::shared_ptr<A> pt2 = std::make_shared<C>();

  auto b1 = std::dynamic_pointer_cast<B>(pt1);
  auto b2 = std::dynamic_pointer_cast<B>(pt2);

  if (b1)
    std::cout << b1->class_name() << " " << b1->b << std::endl;
  else
    std::cout << "b1 null" << std::endl;

  if (b2)
    std::cout << b2->class_name() << " " << b2->b << std::endl;
  else
    std::cout << "b2 null" << std::endl;

  return 0;
}
```

输出：

```bash
1B 3
b2 null
```





## 3. reinterpret_cast

主要有三种强制转换用途：改变指针或引用的类型、将指针或引用转换为一个足够长度的整形、将整型转换为指针或引用类型。

`reinterpret_cast<type_id> (expression)`

- `type-id` 必须是一个指针、引用、算术类型、函数指针或者成员指针。

- 在使用reinterpret_cast强制转换过程仅仅只是比特位的拷贝，因此在使用过程中需要特别谨慎

```c++
int *a = new int;
double *d = reinterpret_cast<double *>(a);
```





## 4. const_cast

`const` 限定符通常被用来限定变量，用于表示该变量的值不能被修改。而 `const_cast` 则正是用于强制去掉这种不能被修改的常数特性，但需要特别注意的是 `const_cast` 不是用于去除变量的常量性，而是去除指向常数对象的指针或引用的常量性，其去除常量性的对象必须为指针或引用。

`const_cast<type_id> (expression)`

- 该运算符用来修改类型的 `const` 或 `volatile` 属性，` type_id `和 `expression` 的类型是一样的
- 常量指针被转化成非常量指针，并且仍然指向原来的对象
- 常量引用被转换成非常量引用，并且仍然指向原来的对象；常量对象被转换成非常量对象