---
title: constexpr用法
date: 2025-11-04 18:05:48
tags:
- c++
categories:
- c++
keywords:
description:
---



`constexpr`是C++11引入的关键字，用于在编译期确定变量、函数或对象的值，从而实现编译期常量计算。随着C++标准的发展，`constexpr`的用法和功能也不断增强。



---

## 1. C++11: `constexpr`的初步引入



### 1.1 `constexpr`变量

用`constexpr`修饰的变量必须在定义时初始化，且初始化值必须是编译期常量。

```cpp
constexpr int max_size = 100;
```

`max_size`可以用在数组大小、模板参数等需要常量表达式的场景。



### 1.2 `constexpr`函数

`constexpr`函数表示该函数可以在编译期求值，也可以在运行时调用。函数体必须满足编译期求值条件，限制较多（只能有单一`return`语句等）。

```c++
constexpr int square(int x) {
    return x * x;
}

constexpr int val = square(5);  // 编译期计算，val = 25
```



### 1.3 `constexpr`构造函数和对象

- 支持`constexpr`构造函数，可以在编译期创建常量对象。
- 类型必须满足字面类型（Literal Type）。

```c++
struct Point {
    int x, y;
    constexpr Point(int a, int b) : x(a), y(b) {}
};

constexpr Point p(1, 2);
```



### 1.4 `constexpr`成员函数（限制较多）

C++11支持`constexpr`成员函数，但函数体限制较多。C++14以后允许更复杂的函数体。


------



## 2. C++14: `constexpr`功能增强



### 2.1 更灵活的`constexpr`函数

- 允许函数体内使用局部变量、循环、条件语句等复杂逻辑。

```c++
constexpr int factorial(int n) {
    int result = 1;
    for (int i = 2; i <= n; ++i)
        result *= i;
    return result;
}

constexpr int val = factorial(5);  // 120
```



### 2.2 `constexpr`成员函数增强

- 成员函数可以包含更复杂的语句，支持更灵活的编译期计算。

```c++
struct Circle {
    double radius;
    constexpr Circle(double r) : radius(r) {}
    constexpr double area() const {
        return 3.14159 * radius * radius;
    }
};

constexpr Circle c(5.0);
constexpr double a = c.area();
```

------



## 3. C++17: 编译期条件判断与`constexpr Lambda`



### 3.1 新增：`if constexpr`

`if constexpr`是编译期条件判断语句，条件必须是编译期常量表达式。

编译器只编译满足条件的分支，未满足分支代码被忽略，避免无效代码编译错误。

```c++
template<typename T>
void foo(T t) {
    if constexpr (std::is_integral_v<T>) {
        std::cout << "Integral type: " << t << std::endl;
    } 
    // NOTE: 这里的else不要省略改成if里return，否则编译时不判断
    else {
        std::cout << "Non-integral type" << std::endl;
    }
}
```

- 避免无效代码编译错误
- 提高模板代码灵活性
- 优化性能，减少冗余代码



### 3.2 `constexpr Lambda`

Lambda表达式可以声明为`constexpr`，支持编译期求值。

```c++
constexpr auto square = [](int x) { return x * x; };
constexpr int val = square(5);
```

------



## 4. C++20: `constexpr` 生态进一步完善



### 4.1 新增：`consteval`

`consteval`函数必须在编译期求值，不能运行时调用。

```c++
consteval int get_five() {
    return 5;
}

int main() {
    constexpr int a = get_five();  // 合法
    int b = get_five();            // 错误，不能运行时调用
}
```



### 4.2 新增：`constinit`

`constinit`保证变量在程序启动时初始化，但不要求是常量。

```c++
constinit int x = 10;
```



### 4.3 标准库容器支持`constexpr`

`std::array`等容器支持`constexpr`，允许在编译期初始化和操作。

```c++
constexpr std::array<int, 3> arr = {1, 2, 3};
constexpr int sum = arr[0] + arr[1] + arr[2];
```



### 4.4 `constexpr`和`if constexpr`结合使用，实现更灵活的编译期逻辑

```c++
template<typename T>
constexpr T abs_val(T x) {
    if constexpr (std::is_signed_v<T>) {
        return x < 0 ? -x : x;
    } else {
        return x;
    }
}
```

------



## 5. 总结

| 标准版本 | 主要特性及增强                                           |
| -------- | -------------------------------------------------------- |
| C++11    | 引入`constexpr`变量、函数、构造函数，支持简单编译期计算  |
| C++14    | 放宽`constexpr`函数限制，支持复杂语句和成员函数          |
| C++17    | 新增`if constexpr`编译期条件判断，支持`constexpr` Lambda |
| C++20    | 引入`consteval`、`constinit`，标准库容器支持`constexpr`  |

- `constexpr`用于实现编译期常量计算，提升性能和安全性。
- 支持变量、函数、构造函数、成员函数、Lambda等多种形式。
- `if constexpr`实现编译期条件分支，极大增强模板编程灵活性。
- C++20引入`consteval`和`constinit`，进一步丰富编译期和初始化语义。
- 标准库容器逐步支持`constexpr`，方便编译期复杂数据结构操作。
