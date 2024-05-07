---
title: Effective C++ 笔记
date: 2022-09-17 15:14:51
# top: 1
tags:
- c++
categories:
- c++
keywords:
- effective
- c++
- coding
description:
---



## 让自己习惯C++

### 1. 视C++为一个语言联邦

C++是从四个次语言组成的联邦政府，每个次语言都有自己的规则。

- C：C++的基础
- Object-Oriented C++：面向对象设计，类/封装/继承/多态/virtual函数等
- Template C++：范型编程/模版元编程
- STL：容器/迭代器/算法/函数对象

对于不同的次语言，往往有不同的高效编程守则。

对于内置类型（C-like）而言 pass-by-value 通常比 pass-by-referenece 高效；对于用户自定义的类型（Object-Oriented C++），因为构造函数和析构函数的存在，pass-by-reference-to-const 往往更好；对于 Template C++，由于所处理的对象的类型不确定，也应使用 pass-by-reference-to-const；对于 STL 的迭代器和函数对象，由于它们都是基于C指针塑造出来的，所以 pass-by-value的守则再次适用。



### 2. 尽量以 const, enum, inline 替代 #define

- #define 不被视为语言的一部分，预处理器会将所有的宏替换，容易导致问题且难以追踪。

- #define 不重视作用域，在被定义后的编译过程中一直有效，所以没有任何封装性，不能用来定义 class 专属常量，也没有 private #define 这样的东西。

  

#### 普通常量替代 #define -> const

```C++
#define ASPECT_RATIO 1.653

---

const double AspectRatio = 1.635;
```



#### 常量指针

由于常量指针通常被放在头文件内，因此有必要将指针也声明为 const。

```C++
const char* const authorName = "Tiffany";
const std::string authorName("Tiffany");
```



#### class 专属常量

为了确保常量只有一份实体，必须让它成为 `static` 成员：

```C++
class A {
private:
	static const int Num = 5;  // 声明式
	int scores[Num];
};
```

```C++
const int A::Num; // 定义式，应写在.cc文件中
```



#### enum hack

旧式编译器有可能不允许 static 成员在声明式上获得初值（只能在定义式时初始化），可以使用 the enum back 补偿做法，将枚举类型的数值当做int使用。

```C++
class A {
private:
  enum { Num = 5 };
  int scores[Num];
};
```



#### template inline 函数替代宏

宏看起来像函数，但不会招致函数调用带来的额外开销，可是非常容易导致不确定行为。使用 template inline 函数可以获得宏带来效率以及一般函数的所有可预料行为和类型安全性。

``` C++
#define CALL_WITH_MAX(a, b) f((a) > (b)) ? (a) : (b))

---

template<typanme T>
inline void call_with_max(const T& a, const T& b) // pass by reference-to-const
{
  f(a > b ? a : b);
}
```



### 3. 尽可能使用 const

#### const 语法

如果 const 出现在星号左边，表示被指物为常量；如果出现在星号右边，表示指针自身是常量（不能指向别的东西，但所指的东西的值可以改动）；如果出现在星号两边，表示被指物和指针都是常量。

```C++
char greeting = "hello";

char* p = greeting;             // 非常量

const char* p = greeting;       // 被指的data为常量
char const* p = greeting;       // 被指的data为常量

char* const p = greeting;       // 指针为常量

const char* const p = greeting; // 指针和被指的data都为常量
```



#### 函数返回值声明为const

```C++
class Rational {...};
const Rational operator* (const Rational& lhs, const Rational& rhs);
```

此处 const 可以避免一些~~傻逼~~行为，比如：

```C++
Rational a, b, c;
if ((a + b) = c) // 奇怪的赋值
```

> 对照查看 [条例10-令 operator= 返回一个 reference to *this](#10-令-operator=-返回一个-reference-to-*this)

#### const 成员函数

- const 对象只能访问其 const 成员函数，所以当对象被以 pass-by-reference-to-const 方式传递时，也只能调用其 const 成员函数。

- 两个函数如果只是常量性不同，也是重载。

- **bitwise const**：成员函数只有在不更改对象的任何变量时才可以说是const。

  ```C++
  class TextBlock {
  public:
    ...
    const char& operator[] (size_t position) const // !!返回也为const
    {
      return text[position];
    }
    char& operator[] (size_t position)
    {
      return text[position];
    }
  private:
    std::string text;
  };
  
  TextBolck tb("Hello"); // non-const TextBlock
  std::cout << tb[0];    // 调用 non-const TextBlock::operator[]
  tb[0] = 'x';           // 没问题
  
  const TextBlock ctb("Hello"); // const Textblock
  std::cout << ctb[0];          // 调用 const TextBlock::operator[]
  ctb[0] = 'x';                 // 错误，不允许写入
  ```

  

  ```C++
  class WrongTextBlock{
  public:
    char& operator[](size_t position) const // 返回值未声明为const
    {
      return text[position];
    }
  private:
    std::string text;
  };
  
  const WrongTextBlock cwtb("Hello");
  cwtc[0] = 'x';     // Hello -> xello
  ```

  

- 将成员变量声明为 mutable，则此变量在 const 成员函数内也可以被修改。

  ```C++
  class TextBlock{
  private:
    std::string text;
    mutable size_t text_length;
    mutable bool length_is_valid;
  public:
    std::size_t length() const
    {
      if (!length_is_valid)
      {
        text_length = text.size();  // mutable members
        length_is_valid = true;     // 可以被修改
      }
      
      return text_length;
    }
  };
  ```

  

- 当 const 和 non-const 成员函数有着实质等价的实现时，令 non-const 版本调用 const 版本（通过转型）避免代码重复。

  ```C++
  class TextBlock {
    cosnt char& operator[] (size_t position) const
    {
      ...
  		return text[position];
    }
  
    char& operator[] (size_t position)
    {
      // static_cast: *this -> const TextBlock&
      // const_cast: const char& -> char&
      return const_cast<char&>(static_cast<const TextBlock&>(*this)[position]);
    }
  };
  ```



### 4. 确定对象被使用前已被初始化

#### 内置类型需要手动初始化

```C++
int x = 0;
const char* test = "A C-style string";
double d;
std::cin >> d;  // 以读取 input stream 的方式完成初始化
```



#### 自定义类型：使用初始化列表

```C++
class PhoneNumber {...};
class ABEntry {
public:
  ABEntry(const std::string& name, const std::string& address, 
          const std::list<PhoneNumber>& phones)
    : the_name(name),
      the_address(address),
      the_phones(phones),
      num_times_consulted(0) 
  {...}
  
  ABEntry() 
    : the_name(),   // 调用the_name的default构造函数
      the_address(),
      the_phones(),
      num_times_consulted(0)
  {}

private:
  std::string the_name;
  std::string the_address;
  std::list<PhoneNumber> the_phones;
  int num_times_consulted;
};
```



#### non-local static 对象

- `static` 对象的寿命从被构造出来直到程序结束为止，包括 ：`global`对象，定义于 `namespace` 作用域内的对象，在 `classes` 内，在函数内，在 `file` 作用域内被申明为 `static` 的对象。
- 函数内的 `static` 对象称为 `local static` 对象，其他称为 `non-local static`  对象。
- 编译单元是指产出单一目标文件的源码，基本上是单一源码文件加上所含的头文件。
- 如果编译单元内的某个 non-local static 对象的初始化动作使用了另一编译单元内的某个 non-local static 对象，后者可能尚未被初始化。**因为C++对 ”定义于不同编译单元内的 non-local static 对象“ 的初始化次序并无明确定义**

```c++
class FileSystem {
public:
  ...
  std::size_t num_disks() const;
};
extern FileSystem tfs;
```

```c++
class Directory {
public:
	Dictory(params)
  {
    ...
    size_t discks = tfs.numsDicks();
  }
  ...
};
```

```C++
Directory temp_dir(params);   // tfs可能还没被初始化
```



- 解决方法（Singleton模式常用实现方法）：将每个 non-local static 对象搬到自己的专属函数内 (在函数内声明为 static)，这些函数返回一个 reference 指向它所含的 static 对象，用户调用这些函数而不直接指涉这些对象。

```C++
class FileSystem {...};
FileSystem& tfs()
{
  static FileSystem fs;
  return fs;
};

class Directory {...};
Directory::Directory(params)
{
  ...
  size_t disks = tfs().numsDisks();
};
Directory& tempDir()
{
  static Directory td;
  return td;
}
```





## 构造 / 析构 / 赋值运算

### 5. 了解C++默默编写并调用哪些函数

- 对于每个 class，如果没有声明，编译期会为它自动声明（编译器版本的）一个 **copy 构造函数**，一个 **copy assignment 操作符**和一个**析构函数**。如果没有声明任何的构造函数，编译器也会自动声明一个 **default 构造函数**。这些函数都是 **public 且 inline** 的。

- 编译器产出的析构函数是 non-virtual 的，除非这个类的基类自身声明有虚析构函数。
- 编译器生成的 copy 构造函数和 copy assignment 操作符，只是单纯地将来源对象的每一个 non-static 成员变量拷贝到目标对象。
- 只有当生出的代码合法且有适当机会证明它有意义，编译器才会自动生成 copy assignment 操作符，任一条件不符合时（比如 class 内含 reference 成员 / const 成员），编译器将拒绝生成 operator=。



### 6. 若不想使用编译器自动生成的函数，就该明确拒绝

- 将默认生成的成员函数声明为 private 且不实现他们，或者继承（可以是 private 继承）Uncopyable 也是一种做法。

- 现在可以使用关键字 delete 阻止编译器自动生成相关函数

  ```C++
  class A {
  public:
    A() = delete;                     // 阻止编译器生成默认构造函数
    A(const A&) = delete;             // 阻止编译器生成copy构造函数
    A& operator=(const A&) = delete;  // 阻止编译器生成copy assignment
  };
  ```

  

### 7. 为多态基类声明 virtual 析构函数

- 当一个派生类对象经由一个基类指针被删除，而该基类带着一个non-virtual析构函数，其结果未有定义——实际执行时通常发生的事对象的 derived 成分没被销毁（造成一个诡异的局部销毁对象）。

- 任何 class 只要带有 virtual 函数都几乎确定应该也有一个 virtual析构函数。

- **多态机制**：在**运行期**决定哪一个 virtual 函数应该被调用。每一个有虚函数的类或有虚函数的类的派生类，都有一个由函数指针构成的数组，存放着所有虚函数的地址，称为**虚函数表 vtbl** (virtual table)；该类的任何对象中都放着一个指向虚函数表的指针 vptr (virtual table pointer)；当对象调用某一 virtual 函数，实际被调用的函数取决于该对象的 vptr 所指的那个 vtbl。

- 由于 vptr 的存在，对象的体积会增加，也就是会增加程序运行时的开销。所以对于不含任何 virutal 函数的类，没必要为它声明 virtual 析构函数。

- 如果希望一个基类成为抽象类，但它本身没有任何纯虚函数，可以为它声明一个**纯虚析构函数**，且（必须）为它提供定义。

  ```C++
  class AWOV {
  public:
    virtual ~AWOV() = 0;
  };
  
  AWOV::~AWOV() {}
  ```

  

### 8. 别让异常逃离析构函数

- 析构函数绝对不要吐出异常，否则很可能造成程序过早结束或出现不明确行为。
- 如果析构函数将调用一个可能抛出异常的函数，析构函数应该捕捉异常，然后吞下它们或结束程序。
- 如果客户需要对某个操作函数运行期间抛出的异常做出反应，那么 class 应该提供一个普通函数（而非在析构函数中）执行该操作



```C++
// 1. 使用 abort() 强迫结束程序
DBConn::DBConn()
{
  try{ db.close(); } // 可能抛出异常
  catch (...)
  {
    // 制作运转记录
    std::abort();
  }
}


// 2. 吞掉异常（只记录不处理）
DBConn::DBConn()
{
  try{ db.close(); } // 可能抛出异常
  catch (...)
  {
    // 制作运转记录
  }
}


// 3. 提供普通函数，客户可以在别的地方调用并处理可能的异常
class DBConn{
public:
  ...
  void close()    // 供客户使用的新函数
  {
    dv.close();
    closed = true;
  }
  
  ~DBConn()
  {
    if (!close)
    {
      try { db.close(); }  // 跟2是一样的 "双保险"
      catch (...)
      {
        // 制作运转记录
      }
    }
  }
  
private:
  DBConnection db;
  bool closed = false;
};
```





### 9. 绝不在构造和析构过程中调用 virtual 函数

- 在派生类对象的基类构造期间，此对象的类型是基类而不是派生类，所以调用 virtual 函数也之后调用到基类对应的函数，不会下降到派生类。同理，对象进入基类的析构函数后，对象也就称为一个基类对象，同样需函数不会下降到派生类。

- 所以绝不要在构造函数和析构函数中调用 virtual 函数，且它们调用的所有函数也都应服从同一约束。

  

### 10. 令 operator= 返回一个 reference to *this

为了实现连锁赋值，赋值操作符必须返回一个reference指向操作符的左侧实参。为classes 实现赋值 (=) 操作符以及所有复制相关运算 (+=, -=, *=等) 时应该遵循此协议：返回一个 reference to *this。

```C++
class Widget{
public:
  ...
  Widget& operator=(const Widget& rhs)
  {
    ...
    return* this;
  }
  
  Widget& operator+=(const Widget& rhs)
  {
    ...
    return* this;
  }
};

// 连锁赋值
Widget a, b, c;
a = b = c = Widget();
```



> 对照查看 [条例3-函数返回值声明为const](#函数返回值声明为const)



### 11. 在 operator= 中处理“自我赋值”

- 验同测试

  ```C++
  class Bitmap {...};
  class Widget
  {
    ...
  private:
    Bitmap* pb;
  };
  
  Widget& Widget::operator=(const Widget& rhs)
  {
    if (this == &rhs) return *this;   // 验同测试
    
    delete pb;
    pb = new Bitmap(*rgs.pb);
    return *this;
  }
  ```

  

- **精心安排的语句**可以使代码具有“异常安全性“（自动获得“自我赋值安全性”）：在复制构造之前别删除原指针

  ```C++
  Widget& Widget::operator=(const Widget& rhs)
  { 
    Bitmap* p_ori = pb;
    pb = new Bitmap(*rgs.pb);
    delete p_ori;             // 复制后再删除原指针
    return *this;
  }
  ```

  

- **copy and swap** 技术

  ```C++
  // 1. 常见
  Widget& Widget::operator=(const Widget& rhs)
  { 
    Widget temp(rhs);
    swap(temp);
    return *this;
  }
  
  // 2. pass by value
  // 将copying从函数本体移到函数参数构造阶段，更高效，但牺牲了清晰性
  Widget& Widget::operator=(Widget rhs)
  { 
    swap(rhs);
    return *this;
  }
  ```

  > [条例25](25. 考虑写出一个不抛异常的 swap 函数) 详细介绍 swap



### 12. 复制对象时勿忘其每一个成分

- Copying 函数应该确保复制 ”对象内的所有成员变量“ 以及 ”所有 base class 成分“。

  ```C++
  class Base {...};
  class Derived
  {
  public:
    Derived(const Derived& rhs);
    Derived& operator=(const Derived& rhs);
    ...
  private:
    int member;
  };
  
  Derived::Derived(const Derived& rhs)
    : Base(rhs),             // 调用base的copy构造函数
      member(rhs.member) {}
  
  Derived::Derived& operator=(const Derived& rhs)
  {
    Base::operator=(rhs);    // 调用base的赋值运算符
    member = rhs.member;
  }
  ```

  

- 不要尝试以一个 copying 函数实现另一个。如果 copy 构造函数和 copy assignment 操作符有相近的代码，消除重复代码的做法是，将重复代码放进一个新的成员函数中 (private init() )，由两个 copy 函数共同调用。





## 资源管理

### 13. 以对象管理资源

- **RAII: Resource Acquisition Is initialization; **获得资源后立刻放进管理对象内。

- 常用的 RAII classes：

  - `auto_ptr` (已被删除) : 复制动作会使他指向null。
  -  **引用计数型智慧指针 (RCSP, reference-counting smart pointer)**  `tr1::shared_ptr` : 持续追踪共有多少对象指向某笔资源，并在无人指向它时自动删除该资源。

  

### 14. 在资源管理类中小心 copying 行为

- 复制**RAII**对象时必须一并复制它所管理的资源。

- 一般资源管理类复制时可以选择以下做法：

  - 禁止复制

    > 参照[条例6](#6-若不想使用编译器自动生成的函数，就该明确拒绝)

  - “引用计数法”（使用 `tr1::shared_ptr` 指定 “删除器” 替代引用次数为0时的删除行为）

    ```C++
    class Lock
    {
    public:
      explicit Lock(Mutex* pm)
        : mutex_ptr(pm, unlock)  // 以unlock作为删除器
      {
        lock(mutex_ptr.get());   // 条例15谈到"get" 
      }
    private:
      std::tr1::shared_ptr<Mutex> mutex_ptr;  // shared_ptr
    };
    ```

    > [条例15](#15-在资源管理类中提供对原始资源的访问)谈到 `get()`

    > [条例18](#18-让接口容易被正确使用，不易被误用) 谈到 定制删除器可以防范DLL问题

  - 复制底层资源（“深度拷贝”）

  - 转移底部资源的拥有权（`auto_ptr`）: 资源的拥有权从被父之物转移到目标物。

    

### 15. 在资源管理类中提供对原始资源的访问

- 许多 APIs 要求直接访问原始资源，所以每一个 RAII class 应该提供一个 “取得其所管理资源” 的办法。
- 对原始资源的访问可能经由显示转换或隐式转换；一般显式转换更安全，而隐式转换对客户更方便。
- `tr1::shared_ptr` 和 `auto_ptr` 都提供一个 `get` 成员函数，用来执行显式转换，也就是返回只能指针内部的原始指针（的复件）。
- `tr1::shared_ptr` 和 `auto_ptr` 也重载了指针取值操作符 (`operator->` 和 `operator*` )，它们允许隐式转换至底层指针。

```C++
// C APIs
FontHandle getFont();
void releaseFont(FontHandle fh);
void changeFontSize(FontHandle f, int newSize);


class Font
{
public:
  explicit Font(FontHandle fh) : f(fh) {}
  ~ Font() { releaseFront(f); }
  ...

  // 1. 显式转换函数
  FontHandle get() const {return f; }
  
  // 2. 隐式转换函数
  operator FontHandle() const
  { return f; }
  
private:
  FontHandle f;
};


Font f(getFont())；
in newFontSize;

// 1. 显示转换
changeFontSize(f.get(), newFontSize);
// 2. 隐式转换
changeFontSize(f, newFontSize);

```



### 16. 成对使用 new 和 delete 时要采取相同形式

new 和 delete 成对， new[] 和 delete[] 成对。

```c++
std::string* stringPtr1 = new std::string;
std::string* stringPtr2 = new std::string[100];

delete stringStr1;
delete [] stringStr2;
```



### 17. 以独立语句将 newed 对象置入智能指针

应以独立语句将 newed 对象存储于智能指针内，否则一旦异常被抛出，有可能导致难以察觉的资源泄漏。

```c++
int priority();
void processWidget(std::tr1::Shared_ptr<Widger> pw, int priority);

// wrong
// 核算实参的次序不确定，有可能：
// 1. new Widget
// 2. 调用priority
// 3. 调用tr1::shared_ptr构造
// 假设priority()调用导致异常，则new Widget返回的指针将会遗失
processWidget(std::tr1::shared_ptr<Widget>(new Widget), priority());

// correct
// 次序确定
std::tr1::shared_ptr<Widget> pw(new Widget);
processWidget(pw, priority());
```





## 设计与声明

### 18. 让接口容易被正确使用，不易被误用

- **"促进正确使用"**的办法包括接口的一致性，以及与内置类型的行为兼容。

- **"防治误用"**的办法包括建立新类型，限制类型上的操作，束缚对象值 (加上 `const` )，以及消除用户的资源管理责任。

  ```C++
  // 创建新类型，避免错误调用
  class Month
  {
  public:
    static Month Jan() {return Month(1);}
    static Month Feb() {return Month(2);}
    ...
  private:
    explicit Month(int m);
    ...
  };
  ```

  > 条例4 谈到 [non-local static对象](#non-local static 对象)的初始化次序问题，以函数代替对象

- `tr1::shared_ptr` 有一个特别好的特质是：它支持定制型删除器，也就是会自动追踪/使用 “每个指针专属的删除器“，可以消除所谓的 "cross-DLL problem"（这个问题发生于 “对象在动态链接程序库(DLL)中被 `new` 创建，却在另一个DLL中被 `delete` 销毁。在许多平台上，这一来跨DLL之 new/delete 成对运用会导致运行器错误。）

  > 参照[条例14](#14-在资源管理类中小心 copying 行为) 定制删除器用来自动解除互斥锁

- `tr1::shared_ptr` 带来性能上的损失(并不显著)：Boost 的 shared_ptr 是原始指针的两倍大，以动态分配内存作为簿记用途和 “删除器之专属数据”，以 virtual 形式调用删除器，并在多线程程序修改引用次数时蒙受线程同步化的额外开销。

  

### 19. 设计 class 犹如设计 type

设计一个高效的 class，往往需要思考以下问题

- 新 type 的对象应该如何被创建和销毁？

- 对象的初始化和对象的赋值应该有什么样的差别？

- 新 type 的对象如果被 passed by value，意味着什么？（copy构造）

- 什么是新 type 的 "合法值" ？（构造函数/赋值操作符/setter函数的约束条件）

- 你的新 type 需要配合某个继承图系吗？（考虑virtual）

- 你的新 type 需要什么样的转换？（显式/隐式转换）

- 什么样的操作符和函数对此新 type 而言是合理的？

- 什么样的标准函数应该驳回？

- 谁该取用新 type 的成员？（关于public/protected/private）

- 什么是新 type 的 "未声明接口" ？（异常安全性？）

- 你的新 type 有多么一般化？（需要template吗）

- 你真的需要一个新 type 吗？

  

### 20. 宁以 pass-by-reference-to-const 替换 pass-by-value

- 效率更高：不需要创建新对象 -> 不需要调用构造函数/析构函数

- 可以避免对象切割 (slicing) 问题：当一个 derived class 对象以 by value 方式传递并被视为一个 base class 对象，那么 base class 的 copy 构造函数会被调用，造成 derived class 部分被切割。

- 但对于内置类型/STL的迭代器和函数对象，pass-by-value往往比较合适。

  

### 21. 必须返回对象时，别妄想返回其 reference

- 函数创建新对象的途径：
  - 在 stack 空间上创建：定义一个 local 变量
  - 在 heap 空间上创建：用 new 创建一个对象
  
- 不要返回 pointer 或 reference 指向一个 local stack 对象：在函数退出之前就被销毁了

- 不要返回 pointer 或 reference 指向一个 heap 对象：违背了 new 和 delete 成对出现的原则，用户不知道如何 delete

- 不要返回 pointer 或 reference 指向一个 local static 对象的引用，否则多次调用时会出现问题（static 只有一份）

- 一个 ”必须返回新对象“ 的函数的正确写法是：就让它返回一个新对象呗

  ```C++
  inline const Rational operator*(const Rational& lhs, const Rational& rhs)
  {
    return Rational(lhs.n * rhs.n, lhs.d * rhs.d);
  }
  ```

  

### 22. 将成员变量声明为private

将成员变量声明为 private可以：

- 使客户只能通过成员函数访问对象（一致性）
- **封装**：将成员变量隐藏在函数接口的背后，可以为实现提供弹性（保留了日后变更实现的权利）



### 23. 宁以 non-member / non-friend 替换 member 函数

```C++
class WebBrowser
{
public:
	...
  void clearCache();
  void clearHistory();
  void removeCookies();
  
  // member function
 	void clearEverything()
  {
    cleraCache();
    clearHistory();
    removeCookies();
  }
};
```

```C++
// webbrowser.h
namespace WebBrowserStuff
{
  class WebBrowser{...};

  // non-member non-friend function
  void clearBrowser(WebBrowser& wb)
  {
    wb.cleraCache();
    wb.clearHistory();
    wb.removeCookies();
  }
}

// webbrowserbookmarks.h
namespace WebBrowserStuff
{
  ...           // 与书签相关的便利函数
}

// webbrowsercookies.h
namespace WebBrowserStuff
{
  ...           // 与cookie相关的便利函数
}
```

- member 函数比 non-member non-friend 函数的封装性低。
- 将所有便利函数放在多个头文件内但是隶属于同一个命名空间，这允许客户只对他们所用的那一小部分系统形成编译相依（降低编译依存性），也可以轻松扩展这一组便利函数。

> [条例31](#31. 将文件间的编译依存关系降至最低) 讨论其他降低编译依存性的做法



### 24. 若所有参数皆需类型转换，请为此采用 non-member 函数

- **建立数值类型**时可以令 classes 支持隐式转换。

  ```C++
  class Rational
  {
  public:
    Rational(int numerator = 0, int denominator = 1); // 允许int->Rational隐式转换
    int numerator() const;
    int denominator() const;
    ...
  };
  ```

- **只有参数列中的参数才是隐式转换的有效参与者**，this 对象无法进行隐式转换。

  ```C++
  // member function
  // wrong
  class Rational
  {
  public:
    const Rational operator*(const Rational& rhs) const;
  };
  
  Rational oneHalf(1,2);
  Rational result;
  result = oneHalf * 2;  // ok
  result = 2 * oneHalf;  // wrong!
                         // -> 2.operator(oneHalf)
                         // -> operator*(2, oneHalf)
                         // 只有后者才在参数列中，可以进行隐式转换
                         // this对象无法进行隐式转换
  ```

  ```C++
  // non-member function
  const Rational operator*(const Rational& lhs, const Rational& rhs)
  {
    return Rational(lhs.numerator() * rhs.numerator(),
                    lhs.denominator() * rhs.denominator());
  }
  
  auto result = 2 * oneHalf; // ok
  ```

  由于有 public 的接口 `numerator()`  `denominator()`，不需要调用 private 的成员变量，所以此 operator* 函数**不需要成为 friend 函数**。

  

### 25. 考虑写出一个不抛异常的 swap 函数

- `swap()` 是**异常安全性编程的脊柱**，也是用来**处理自我赋值可能性的一个常见机制**。

> [条例29](#29-为 "异常安全" 努力是值得的) 讨论异常安全性编程
>
> [条例11](#11. 在 operator= 中处理"自我赋值") 谈到自我赋值的处理

- pimpl (pointer to implementation) 手法

  ```C++
  class WidgetImpl
  {
  public:
    ...
  private:
    int a, b, c;
    std::Vector<double> v;
    ...
  };
  
  class Widget
  {
  public:
    Widget(const Widget& rhs);
    Widget& operator=(const Widget& rhs)
    {
      ...
      *pImpl = *(rhs.pImpl);
    }
    ...
  private:
    WidgetImpl* pImpl;
  };
  ```

- 只要类型支持copying，缺省情况下 swap 动作可由标准程序库提供的 swap 算法完成。但对于某些类型而言，没有必要深拷贝来进行 swap。比如 Widget，只需要置换其 pImpl 指针就可以完成置换。像这样 **std::swap 对自定义类型效率不高时，可以将 std::swap 针对 Widget 特化**（提供一个 member swap 和一个 non-member swap 来调用前者）。

  ```C++
  class Widget
  {
  public:
    ...
    void swap(Widget& other)
    {
      using std::swap;
      swap(pImpl, other.pImpl);  // 编译器将寻找适合的swap版本
    }
  };
  
  namespace std
  {
  	template<>                                // template<> 表示全特化版本
    void swap<Widget>(Widget& a, Widget&b)    // 针对Widget
    {
      a.swap(b);                              // 避免直接调用private成员
    }
  }
  ```

- 对于 class templates (假设 Widget 和 WidgetImpl 都是 class templates)，还是声明一个 non-member swap 让它调用 member swap，但不再将那个 non-member swap 声明为 std::swap 的特化版本。

  ```C++
  namespace WidgetStuff               // 不在std里
  {
    template<typename T>
    class Widget {...};
    ...
      
    template<typename T>
    void swap(Widget<T>&a, Widget<T>&b)
    {
      a.swap(b);
    }
  }
  ```

- C++只允许对 class templates 偏特化，不允许偏特化一个 function template。

- 调用 swap 时应使用 `using std::swap;` 使标准库版本曝光，然后不带任何命名空间资格修饰地调用 swap。

- **成员版本 swap 绝不可抛出异常**，因为 swap 的一个最好的应用是帮助 classes 和 class templates 提供强烈的异常安全性保障。 swap 的缺省版本时以 copying 为基础的，一般情况下 copy 构造和copy assignment 操作符都允许抛出异常。因此当写下一个自定义版本的 swap，往往提供的不只是高效置换对象的办法，而且不抛出异常。



## 实现

### 26. 尽可能延后变量定义式的出现时间

- 定义变量需要承担一次构造函数和一次析构函数的时间。假如该变量未被使用，那么构造函数和析构函数的时间就白白浪费了。尤其是在可能发生异常的函数中，假如你过早的定义变量，然后在你使用这个变量之前抛出了异常，那么这个变量的构造函数就没有意义而且降低效率。所以应该尽可能延后变量定义得时间，只有真正使用这个变量的时候才定义它。

- 不只应该延后变量定义直到非得使用该变量的前一刻为止，甚至应该尝试延后这份定义直到能够给它初值实参为止，这样可增加程序的清晰度并改善程序效率。
- copy construction的效率 > default construction + assign function，所以最好的做法是直接调用copy construction函数对变量直接进行初始化，而不是先定义，再赋值。



### 27. 尽量少做转型动作

- C++提供4种新式转型：尽量使用新式转型替换旧式转型

  - `const_cast<T>()`：用来将对象的常量性转除。
  - `dynamic_cast<T>()`：用来执行 “安全向下转型”，无法由旧式语法执行，但可能耗费重大运行成本。
  - `reinterpret_cast<T>()`：执行低级转型，实际动作及结果可能取决于编译期，所以不可移植。例如将一个 pointer to int 转型为 int，很少见。
  - `static_cast<T>()`：用来强迫隐式转换，比如将 non-const 对象转为 const 对象，将 int 转为 double 等。

- 在注重效率的代码中避免 dynamic_cast，一般用虚函数的方式来避免转型。

  

### 28. 避免返回 handles 指向对象内部成分

**Reference、指针和迭代器都是所谓的handles。**

- 如果返回一个 “代表对象内部数据” 的handle，就可以通过这个修改类内的 private 成员，带来降低对象封装性的风险。这一点可以通过给返回类型加 const 修饰符来防止内部成员变量被修改 (只让渡读取权，而禁止涂写权)。

  ```C++
  class Point
  {
  	Point(int x, int y);
    ...
  };
  
  struct RectData
  {
    Point ulhc;  // upper left-hand corner
    Point lrhc;  // lower right-hand corner
  };
  
  class Rectangle
  {
  public:
    ...
    const Point& upperLeft() const { return pData->ulhc; }
    const Point& lowerRight() const { return pData->lrhc; }
    ...
  private:
    std::tr1::shared_ptr<RectData> pData;
  };
  ```

- 另外可能导致 **dangling handles** (handles 所指东西不再存在)。如果获得的类内的一个成员的handle，但在使用之前，对象被释放了，那么这个handle就指向一个不再存在的对象，handle也就变成虚吊的 (dangling)，会导致 core dump 错误。



### 29. 为 “异常安全” 努力是值得的

**异常安全函数：即使发生异常也不会泄漏资源，不允许任何数据结构败坏。**

- 异常安全函数提供以下三个保证之一：
  - 基本承诺：如果异常被抛出，程序内的任何事物仍然保持在有效状态下。
  - 强烈保证：如果异常被抛出，程序状态不改变。如果函数成功，就是完全成功；如果函数失败，程序会恢复到调用函数之前的状态。
  - 不抛掷保证：承诺绝不抛出异常。

- 强烈保证往往能够以 **copy-and-swap** 实现出来：为你打算修改的对象做出一份副本，然后在那副本身上做一切必要修改。若有任何修改动作抛出异常，原对象仍然保持未改变状态。待所有改变都成功之后，再将修改过的那个副本和原对象在一个不抛出异常的操作中置换(swap)。但并非所有函数都可实现强烈保证，或不具备现实意义（拷贝效率较低）。

  > [条例25](#25. 考虑写出一个不抛异常的 swap 函数) 有介绍 swap 的实现

- 撰写新代码或修改旧码时，首先是 “[以对象管理资源](#13-以对象管理资源)” 阻止资源泄漏，然后挑选三个 “异常安全保证” 中现实可实施的最强烈等级，将它实施于每一个函数身上。

  

### 30. 透彻了解 inlining 的里里外外

- 编译器最优化机制通常被设计用来浓缩那些 “不含函数调用” 的代码，所以当你 inline 某个函数，或许编译期就因此有能力对它执行语境相关最优化。
- inline 函数背后的整体观念是：**将对此函数的每一个调用，都以函数本体替换。**
- 明确声明 inline 函数的做法是在其定义式前加上关键字 inline，隐喻方法是将函数定义于 class 定义式内。
- **inline 只是对编译器对一个申请，不是强制命令，**大部分编译器拒绝太复杂（例如带有循环或者递归）的函数 inlining。

- inline 函数通常只能放在头文件里，因为大多数建置环境 (build environments) 在编译过程中进行 inlining，而为了将一个函数调用替换为被调用函数的本体，编译器必须知道那个函数长什么样。
- templates 通常也只能放在头文件内，因为它一旦被使用，编译器为了将它具现化，需要知道它长什么样。但 template 的具现化和 inlining 无关，所以不要因为 function templates 出现在头文件就将它们声明为 inline。
- **编译器通常拒绝将virtual 函数 inlining**，因为 virtual 意味着 “等待，直到运行期才确定调用哪个函数”，而 inline 意味着 “执行前，先将调用动作替换为调用函数的本体”。
- 构造函数和析构函数也是 inlining 的糟糕候选人，即使它根本不含任何代码。因为C++对于 “对象被创建和被销毁时发生什么事” 做了各式各样的保证。当你创建一个对象时，其每一个 base class 及每一个成员变量都会被自动构造；当你销毁一个对象时，反向程序的析构行为亦会自动发生；如果有个异常在对象构造期间被抛出，该对象已构造好的那一部分会被自动销毁——这些都由编译器于编译期间自动产生的代码执行，这些代码有时候就放在构造函数和析构函数中。
- inline 的弊端：
  - 可能造成程序体积太大
  - 无法随着程序库的升级而升级：一旦需要改变一个 inline 函数 f，所有用到 f 的客户端程序都必须重新编译。（如果 f 是 non-inline 函数，客户端只需要重新连接就可以。如果采取动态链接，改动甚至可以不知不觉地被应用程序吸纳。）
  - 难以调试（无法在一个并不存在的函数内设立断点）



### 31. 将文件间的编译依存关系降至最低

支持 ”编译依存性最小化“ 的一般构想是：依赖声明式，而不要依赖定义式。基于此构想的两个手段是 Handle classes 和 Interface classes。它们解除了接口和实现之间的耦合关系，从而降低了文件间的编译依存性。

当然这两种方式都存在一定的代价：Handle classes 的实现要多分配指针大小的内存，每次访问都是间接访问。Interface classes 的实现方式要承担虚函数表的代价以及运行时的查找表的代价。但是一般这两种实现对资源和效率的影响通常不是最关键的，因此可以放心的使用，类似 tensorflow 源码中就大量使用这种方式降低编译依赖。



#### Handle Class

```C++
// person.hh

#include <memory>

class PersonImpl;   // 声明式，不需要include它的头文件

class Person        // handle class
{
public:
  Person(const std::string& name);

  std::string name() const;

private:
  // 因为是指针，编译器不需要知道PersonImpl的大小
  // 只需要分配指针需要的内存
  std::shared_ptr<PersonImpl> impl_;
};
```

```C++
// person_impl.hh

class PersonImpl  // 实现类，完成实际工作
{
public:
  PersonImpl(const std::string& name) : name_(name) {}
  std::string name() const
  {
    return name_;
  }

private:
  std::string name_;
};
```

```C++
// person.cc
#include "person.hh"
#include "person_impl.hh"

class PersonImpl;

Person::Person(const std::string& name)
  :impl_(std::make_shared<PersonImpl>(name))
{}

std::string Person::name() const
{
  return impl_->name();
}
```

像 Person 这样使用 **pimpl idiom** 的类，称为 **Handle classes**。它们将所有函数转交给相应的实现类并由后者完成实际工作。这样的调用并不会改变它做的事，只会改变它做事的方法。

此时 Person 类头文件只使用了实现类 (PersonImpl) 的声明式，所以实现类的任何变化并不会导致 Person 类头文件需要重新编译，因此所有只 include 了 person.hh 的文件也都不需要重新编译了，这样就大大降低了文件之间的编译依存关系。


#### Interface Class

可以令 Person 成为一种特殊的 abstract base class，称为 **Interface class**。这种类的目的是详细一一描述 derived classes 的接口，因此它通常不带成员变量，也没有构造函数，**只有一个虚析构函数以及一组纯虚函数**，用来叙述整个接口。客户通过 **factory 函数** (或称为 virtual 构造函数，往往在 interface class 内被声明为 static) 来为这种类创建新对象。

```C++
// person.hh

class Person
{
public:
  virtual ~Person();
  virtual std::string name() const = 0;
  ...
  static std::shared_ptr<Person> create(const std::string& name);
};
```

```C++
// real_person.hh

#include "person.hh"

class RealPerson : public Person
{
public:
  RealPerson(const std::string& name):name_(name) {}
  virtual ~RealPerson() {}
  std::string name() cosnt { return name_; }
private:
  std::string name_;
};
```

```C++
// person.cc

#include "person.hh"
#include "real_person.hh"

std::shared_ptr<Person> Person::create(const std::string& name)
{
  return std::make_shared<RealPerson>(name);
}
```





## 继承与面向对象设计

### 32. 确定你的 public 继承塑模出 is-a 关系

"public继承" 意味 is-a。适用于 base class 身上的每一件事情一定也适用于 derived class 身上，因为每一个 derived class 对象也都是一个 base class 对象。



### 33. 避免遮掩继承而来的名称

子类会遮掩父类同名的函数（这其实违反了 public 继承所暗示的 is-a 关系），可以使用 **using 声明式**或者**inline转交函数**避免。



### 34. 区分接口继承和实现继承

- 接口继承和实现继承不同。
  - pure virtual 函数只指定接口继承
  - impure virtual 函数指定接口继承和缺省实现继承
  - non-virtual 函数指定接口继承及强制实现继承
- 将一个 impure virtual 函数拆成一个 pure virtual 函数 和一个 non-virtual 函数，可以实现接口和默认实现的分离。

- pure virtual 函数可以拥有定义式，但调用它的唯一途径是调用时明确指出其 class 名称。通过这种方式也可以实现接口和默认实现分离，但是这样两者就无法拥有不同保护级别了。



### 35. 考虑 virtual 函数以外的选择

讨论例子的原版

```C++
class GameCharacter
{
public:
  virtual int healthValue() const;
  ...
};
```



#### 由 non-virtual interface (NVI) 手法实现 template method 模式

**NVI 手法:** 通过 public non-virtual 成员函数间接调用 private virtual 函数。这是所谓 **template method** 设计模式的一种独特表现形式。这种手法允许子类重新定义虚函数，从而赋予它们 ”如何实现机能“ 的控制能力， 但父类保留 ”函数如何被调用“ 的权利。

```C++
class GameCharacter
{
public:
  int healthValue() const
  {
    ...														// pre-process
    int reVal = doHealthValue();  // process
    ...														// post-process
    return reVal;
  }
 	...
private:
  virtual int doHealthValue() const;
}
```



#### 由 function pointers 实现 strategy 模式

```C++
int defaultHealthCalc(const class GameCharacter& gc);

class GameCharacter
{
public:
  typedef int (*HealthCalcFunc)(const GameCharacter&);
  explicit GameCharacter(HealthCalcFunc hcf = defaultHealthCalc) // 传入计算的函数指针
 		: healthFunc(hcf) {}                                         // 且有默认值
  int helthValue() const { return healFunc(*this); }
  ...
private:
  HealthCalcFunc helthFunc;
}
```

传入函数指针替代虚函数，好处是可以进行运行期变更 (可以提供一个 setHealthCalcFunc 函数)，缺点是可能会牺牲封装性(需要声明non-member 的计算函数为 friend 以访问成员变量或将某一部分提供 public 访问函数)。



#### 由 tr1::function 完成 strategy 模式

将前者的函数指针改为一个类型为 tr1::function 的对象。

```C++
// 原用函数指针
typedef int (*HealthCalcFunc)(const GameCharacter&);

// tr1::function
typedef std::tr1::function<int (const GameCharacter&)> HEalthCalcFunc;
```

目前这个签名式代表的函数是 “接受一个 reference 指向 const GameCharacter，并返回 int”。这个 tr1::function 类型产生的对象可以持有任何与此签名式兼容的可调用物 (函数指针/函数对象/成员函数指针)。所谓兼容，就是这个可调用物的参数可被隐式转换为 `const GameCharacter&`，而其返回类型可被隐式转换为 `int`。

```C++
// 以下都是兼容的可调用物

short calHealth(const GameCharacter&);

struct HealthCalculator{
  int operator()(const GameCharacter&) const
  { ... }
};

class GameLevel
{
public:
  float health(const GameCharacter&) const;
  ...
}

// 调用
GameCharacter gc1(calHealth);

GameCharacter gc2(HealthCalculator());

GameLevel currentLevel;
GameCharacter gc3(std::tr1::bind(&GameLevel::health, currentLevel, _1));
```



#### 古典的 strategy 模式

Strategy 设计模式的传统实现手法是：将继承体系内的 virtual 函数替换为另一个继承体系内的 virtual 函数。

```C++
// HealthCalcFunc 继承体系
class HealthCalcFunc
{
public:
  virtual int calc(const class GameCharacter& gc) const;
  ...
};
class SlowHealthLoser : public HealthCalcFunc {...};


HealthCalcFunc defaultHealthCalc;

// GameCharacter 继承体系
class GameCharacter
{
public:
  explicit GameCharacter(HealthCalcFunc* phcf = &defaultHealthCalc)
    : pHealthCalc(phcf) {}
  int healthValue() const { return pHealthCalc->clac(*this); }
  ...
private:
  HealthCalcFunc* pHealthCalc;
};
class EvilBadGuy : public GameCharacter {...};
```



### 36. 绝不重新定义继承而来的 non-virtual 函数

绝对不要重新定义继承而来的 non-virtual 函数。non-virtual 在实现上是**静态绑定**的，调用父类还是子类的函数完全取决于指针或者对象的类型。在子类重定义 non-virtual 时，父类的相同的函数是不会被覆盖的。



### 37. 绝不重新定义继承而来的(virtual函数的)缺省参数值

- virtual 函数是动态绑定 (后期绑定) 的，而缺省参数值是静态绑定 (前期绑定) 的。所以可能会 “调用一个定义于 devired class 内的 virtual 函数” 的同时，却使用 base class 为它制定的缺省参数值。

- virtual 函数不应该带有缺省值，可以拆分成一个带有缺省值的 non-virtual 函数和一个不带缺省值的 virtual 函数，在前者内部调用后者，也就是条例35中介绍的[NVI 手法](#由 non-virtual interface (NVI) 手法实现 template method 模式)。

  

### 38. 通过复合塑模出 has-a 或 “根据某物实现出”

- 复合是类型之间的一种关系，也就是某种类型的对象内含另一种类型的对象。

- 复合的意义和 public 继承完全不同。

- 在应用域 (程序中的对象相当于你所塑造的世界中的某些事物，如人、车辆等)，复合意味着 has-a。

- 在实现域 (实现细节上的人工制品，如缓冲区、互斥器、查找树等)，复合意味着 is-implemented-in-terms-of (根据某物实现出)。

  

### 39. 明智而审慎地使用 private 继承

- 对于 private 继承关系，编译器不会自动将一个 derived class 对象转换为一个 base class 对象。
- 由 private base class 继承而来的所有成员，在 derived class 对象中都会变成 private 属性。
- private 继承纯粹是一种实现技术，它意味着只有实现部分被继承，而接口部分应略去。
- private 继承也意味着 is-implemented-in-terms-of。
- 尽可能使用复合，必要时才使用 private 继承。一般当 derived class 需要访问 base class 的 protected 成员时，或需要重新定义继承而来的 virtual 函数时，才选择使用 private 继承。
- private 继承可以造成 empty base 最优化。



### 40. 明智而审慎地使用多重继承

- 多重继承 (multiple inheritance, MI) 的意思是继承一个以上的 base classes

- 可能导致歧义 (从多个基类中继承相同名称)，需要明确指出要调用的函数

  ```c++
  class BorrowableItem{
  public:
    void checkOut();
    ...
  };
  class ElectronicGadget{
  private:
    bool checkOut() const;
    ...
  };
  class MP3Player: public BorrowableItem,
  								 public ElectronicGadget
  {...};
  
  MP3Player mp;
  mp.BorrowableItem::checkOut();  // 明确指出要调用的函数
  ```

- 可能导致要命的 “钻石型多重继承”

  ```plantuml
  @startuml
  File <|-- InputFile
  File <|-- OutputFile
  InputFile <|-- IOFile
  OutputFile <|-- IOFile
  @enduml
  ```

- 为了解决多继承时的命名冲突和冗余数据问题，C++ 提出了**虚继承**，使得在派生类中只保留一份间接基类的成员。虚继承的目的是让某个类做出声明，承诺愿意共享它的基类，被共享的基类就称为**虚基类**。在这种机制下，不论虚基类在继承体系中出现了多少次，在派生类中都只包含一份虚基类的成员。

- InputFile 和 OutputFile 必须采用虚继承，否则将导致 IOFile 类中保留两份 File 类的成员。

- 虚继承会增加大小、速度、初始化复杂度等成本。

- 多重继承可用于：public 继承某个 Interface class + private 继承某个协助实现的 class。





## 模版与范型编程

### 41. 了解隐式接口和编译期多态

- 面对对象编程总是以显式接口和运行时多态来解决问题，而在templates及泛型编程的世界中，隐式接口和编译期多态则更重要。
- 显式接口由函数的签名式 (也就是函数名称、参数类型、返回类型)构成。
- 隐式接口则并不基于函数签名式，而是由有效表达式组成。T 必须支持哪一组接口，由 template 中执行于 T 身上的操作来决定。
- 编译期多态：以不同的 template 参数具现化 function templates，会导致调用不同的函数 (函数重载解析)，而这样的具现行为发生在编译期。



### 42. 了解 typename 的双重意义

- 用于声明 template 类型参数时，class 和 typename 的意义完全相同，可以互换。

  ```c++
  template<class T> class Widget;
  template<typename T> class Widget;
  ```

- template 内出现的名称如果相依于某个 template 参数，那么这个名称称为从属名称 (dependent names)；如果从属名称在 calss 内呈嵌套状，则称为嵌套从属名称 (nasted dependent names)；而不依赖于任何template 参数的名称称为非从属名称 (non-dependent names)。

- 编译器在 template 中遇到嵌套从属名称时，将**默认此名称不是一个类型** (比如 `T::iterator` 有可能是个静态变量或者T namespace中的变量)，除非我们用关键字 **typename** 指明。

  ```c++
  template <typename T>
  void print2nd(const T& container)
  {
    if (container.size() >= 2)
    {
      // 用typename说明T::const_iterator是一个类型
      typename T::const_iterator iter(container.begin());
      ...
    }
  }
  ```

- 类的**继承列表**和**初始化列表**中的类型不需要 typename 指定类型，因为继承的一定是一个类，而初始化列表一定是调用父类的构造或者初始化某个成员。

  ```c++
  template <typename T>
  class Derived: public Base<T>::Nested         // 继承列表: 不需要typename
  {
  public:
    explict Derived(int x) : Base<T>::Nested(x) // 初始化列表: 不需要typename
    {
      typename Base<T>::Nested temp;            // 需要用typename
      ...
    }
  }
  ```

- 配合 **typedef**使用
  
  ```C++
  template <typename IterT>
  void workWithIterator(IterT iter)
  {
    typedef typename std::iterator_traits<IterT>::value_type value_type;
    value_type temp(*iter);
  }
  ```



### 43. 学习处理模版化基类内的名称

在 `derived class templates` 内直接调用 `base class` 函数将无法通过编译。

```c++
template<typename T>
class Base
{
	public:
    ...
    void print()
    {...}
};

template<typename T>
class Derived : public Base<T>
{
  public:
    ...
    void print_log()
    {
      ...
      print();    // 调用base class函数，无法通过编译
    }
}
```

这是由于 `base class templates` 有可能被特化，而那个特化版本可能并不提供和一般性 `template` 相同的接口，因此编译器拒绝在 `templatized base classes` 内寻找继承而来的名称 (例子中的 `print` )。

```c++
class ClassA {};

// 如果针对ClassA产生一个Base特化版，它可以不提供某些一般性的接口
template<>
class Base<ClassA>
{
  public:
   ...
   //  沒有提供 print()
}
```



有3种方式使其进入`templatized base classes` 内寻找（实际上都是对编译器承诺 base class template 的任何特化版本都将支持一般版本所提供的接口，如果后续使用时违反此承诺仍将无法通过编译）：

1. `this->` 调用

   ```c++
   this->print();
   ```

2. `using` 声明式

   ```c++
   template<typename T>
   class Derived : public Base<T>
   {
     public:
       ...
       using Base<T>::print;
       void print_log()
       {
         ...
         print();    // 调用base class函数，无法通过编译
       }
   }
   ```

3. 明确指出被调用的函数位于 `base class` 内

   ```c++
   Base<T>::print();
   ```

   但这种方式由于需要明确资格修饰，无法进行 `virtual` 绑定。



### 44. 将与参数无关的代码抽离 templates

- Templates 会根据具体类型具像化多份代码，如果将与模板无关的代码也放入模板函数或者类中，可能会导致代码膨胀：其二进制码带着重复的代码/数据。
- 任何template代码都不应该与某个造成膨胀的template参数产生相依关系。
- 在 non-template 代码中，重复十分明确，你可以明显看到两个函数或两个 classes 之间有所重复；但在 template 代码中，重复往往是隐晦的，因为只存在一份 template 源码，当 template 被具现化多次时却可能发生重复。

```c++
// 类型参数T 非类型参数n
template<typename T, size_t n>
class SquareMatrix
{
  public:
  ...
  void invert();
};

SquareMatrix<double,5> sm1;
sm1.invert();

SquareMatrix<double,10> sm2;
sm2.invert();

// 上述代码具现化了两份invert函数，它们并非完全重复，因为其中一个操作的是5*5的矩阵而另一个操作的是10*10的矩阵，但除了常量5和10，两个函数的其他部分完全相同。这就造成了代码膨胀。
```

- 函数模板中与参数无关的代码可以包装成单独的函数，类模板中与参数无关的模板可以放到父类中。

- 因非类型模版参数而造成的代码膨胀，往往可以用函数参数或class成员变量替换template参数来消除。

- 因类型参数而造成的代码膨胀，往往可以通过带有完全相同二进制表述的具现类型共享实现码来降低。比如 `list<int*>` , `list<const int*>` 等使用指针的情况，往往应该对每个成员函数使用唯一一份底层实现。所以，对于操作强型指针 ( `T*` ) 的成员函数，应该让它们调用另一个操作无类型指针 (void *) 的函数，由后者来完成实际工作。

  

### 45. 运用成员函数模版接受所有兼容类型

### 46. 需要类型转换时请为模版定义非成员函数

### 47. 请使用 traits classes 表现类型信息

### 48. 认识 template 元编程



## 定制 new 和 delete [暂时不做总结]

### 49. 了解 new-handle 的行为

### 50. 了解 new 和 delete 的合理替换时机

### 51. 编写 new 和 delete 时需要固守常规

### 52. 写了 placement new 也要写 placement delete



## 杂项讨论

### 53. 不要轻忽编译期的警告

### 54. 让自己熟悉包括 TR1 在内的标准程序库

### 55. 让自己熟悉boost
