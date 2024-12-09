---
title: std::enable_if的使用
date: 2024-12-09 18:23:37
tags:
- std
- c++
categories:
- c++
keywords:
description:
---

`std::enable_if` 是 `C++` 中的一种 `SFINAE`（Substitution Failure Is Not An Error）工具，用于在模板中启用或禁用某些函数或类的特化。它通常用于条件编译，允许我们根据模板参数的某些特性（如类型是否满足某些条件）来选择性地启用或禁用某些代码。



## 基本语法

1. `std::enable_if` 定义在 `<type_traits>` 中，其基本形式如下：

    ```c++
    template<bool B, class T = void>  
    struct enable_if {};
    ```

    - 如果 `B` 为 `true`，`std::enable_if` 定义一个类型 `T`（默认为 `void`）。
    - 如果 `B` 为 `false`，`std::enable_if` 不定义任何类型，从而导致模板匹配失败（`SFINAE`）。

1. 可以使用 `std::enable_if_t` 简化代码：

  ```c++
  template<bool B, class T = void>  
  using enable_if_t = typename enable_if<B, T>::type;
  ```



## 使用场景

### 函数模板的启用/禁用

通过 `std::enable_if`，我们可以根据模板参数的某些条件，选择性地启用或禁用某些函数模板。

```c++
#include <iostream>  
#include <type_traits>  

// 当T是整数类型时启用此函数  
template <typename T>  
typename std::enable_if<std::is_integral<T>::value, void>::type  
print(T value) {  
    std::cout << "Integer: " << value << std::endl;  
}  

// 当T不是整数类型时启用此函数  
template <typename T>  
typename std::enable_if<!std::is_integral<T>::value, void>::type  
print(T value) {  
    std::cout << "Not an integer: " << value << std::endl;  
}  

int main() {  
    print(42);          // 调用第一个函数，输出 "Integer: 42"  
    print(3.14);        // 调用第二个函数，输出 "Not an integer: 3.14"  
    return 0;  
}
```

- `std::is_integral<T>::value` 是一个类型特性，用于判断 `T` 是否是整数类型。
- `std::enable_if` 根据这个条件启用或禁用对应的函数模板。

### 使用 `enable_if_t` 简化代码

在 C++14 中，可以使用 `std::enable_if_t` 简化代码，避免显式写 `typename`。

```c++
#include <iostream>
#include <type_traits>

// 仅当 T 是整数类型时启用此函数
template <typename T>
std::enable_if_t<std::is_integral<T>::value, void>
print(T value) {
    std::cout << "Integer: " << value << std::endl;
}

// 仅当 T 不是整数类型时启用此函数
template <typename T>
std::enable_if_t<!std::is_integral<T>::value, void>
print(T value) {
    std::cout << "Not an integer: " << value << std::endl;
}

int main() {  
    print(42);          // 调用第一个函数，输出 "Integer: 42"
    print(3.14);        // 调用第二个函数，输出 "Not an integer: 3.14"
    return 0;
}
```



### 构造函数的启用/禁用

`std::enable_if` 也可以用于类的构造函数中，选择性地启用某些构造函数。

```c++
#include <iostream>
#include <type_traits>

class MyClass {
public:
    // 当T是整数类型时启用此构造函数
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    MyClass(T value) {
        std::cout << "Constructed with integer: " << value << std::endl;
    }

    // 当T不是整数类型时启用此构造函数
    template <typename T, typename = std::enable_if_t<!std::is_integral<T>::value>>
    MyClass(T value) {
        std::cout << "Constructed with non-integer: " << value << std::endl;
    }
};

int main() {
    MyClass obj1(42);      // 调用第一个构造函数，输出 "Constructed with integer: 42"
    MyClass obj2(3.14);    // 调用第二个构造函数，输出 "Constructed with non-integer: 3.14"
    return 0;
}
```

- 通过模板参数 `typename = std::enable_if_t<...>`，我们可以根据类型特性选择性地启用某些构造函数。
- `std::enable_if_t` 的默认模板参数（`typename = ...`）不会影响函数签名，因此可以用于构造函数的重载。

### 类模板的部分特化

`std::enable_if` 也可以用于类模板的部分特化，选择性地启用某些类模板。

```c++
#include <iostream>
#include <type_traits>

// 主模板
template <typename T, typename Enable = void>
class MyClass;

// 特化：当T是整数类型时启用
template <typename T>
class MyClass<T, std::enable_if_t<std::is_integral<T>::value>> {
public:
    void print() {
        std::cout << "Integer specialization" << std::endl;
    }
};

// 特化：当T不是整数类型时启用
template <typename T>
class MyClass<T, std::enable_if_t<!std::is_integral<T>::value>> {
public:
    void print() {
        std::cout << "Non-integer specialization" << std::endl;
    }
};

int main() {
    MyClass<int> obj1;      // 匹配整数特化
    obj1.print();           // 输出 "Integer specialization"

    MyClass<double> obj2;   // 匹配非整数特化
    obj2.print();           // 输出 "Non-integer specialization"

    return 0;
}
```

- 主模板 `MyClass` 是一个空模板，只有在特化时才会启用。
- 使用 `std::enable_if_t` 根据类型特性选择性地启用某些特化。