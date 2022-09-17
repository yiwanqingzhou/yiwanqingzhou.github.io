---
title: 实现strStr()
date: 2022-09-17
tags:
- coding
- algorithm
- c++
categories:
- algorithm
hidden: true
---



# 28.实现strStr() [简单]



## 题目说明

[Leetcode题目链接](https://leetcode-cn.com/problems/implement-strstr/)

实现 [strStr()](https://baike.baidu.com/item/strstr/811469) 函数。

给你两个字符串 `haystack` 和`needle` ，请你在 `haystack` 字符串中找出 `needle` 字符串出现的第一个位置（下标从 0 开始）。如果不存在，则返回  -1 。



**说明：**

当 `needle` 是空字符串时，我们应当返回什么值呢？这是一个在面试中很好的问题。

对于本题而言，当 `needle` 是空字符串时我们应当返回 0 。这与 C 语言的 `strstr()` 以及 Java 的 `indexOf()` 定义相符。



**示例 1：**

```
输入：haystack = "hello", needle = "ll"
输出：2
```

**示例 2：**

```
输入：haystack = "aaaaa", needle = "bba"
输出：-1
```

**示例 3：**

```
输入：haystack = "", needle = ""
输出：0
```



**提示：**

- `0 <= haystack.length, needle.length <= 5 * 10000`
- `haystack` 和 `needle` 仅由小写英文字符组成



## 解题思路

### KMP算法

九章说不考，暂缓



### 暴力枚举

遍历 `haystack` 的每个字符，以其为起点，后面的`needle.size()` 个字符和 `needle` 逐一比较。

- 时间复杂度：O(n*m)



```C++
class Solution {
public:
    int strStr(string haystack, string needle) {
        if (needle.empty())
        {
            return 0;
        }

        int needle_size = needle.size();
        for (int i = 0; i < haystack.size(); i++)
        {
            if (haystack.size() - i < needle_size)
            {
                break;
            }

            bool equal = true;

            for (int j = 0; j < needle.size(); j++)
            {
                if (haystack[i + j] != needle[j])
                {
                    equal = false;
                    break;
                }
            }

            if (equal)
            {
                return i;
            }
        }

        return -1;
    }
};
```



### Rabin-Karp算法

在枚举中，每次都需要比较`needle`的每一个字符，那么如果可以把字符串比较转化为数值比较，就可以加速整个比较过程 —— 使用 `Hashcode` 。

```
hashcode("abcd") = a * 31^3 + b * 31^2 + c * 31^1 + d

// abcd - > bcde
hashcode("abcde") = hashcode("abcd") * 31 + e
hashcode("bcde") = hashcode("abcde") - a * 31^4
```

另外需要一个足够大的 `BASE` 用于取余。

- 时间复杂度：O(n+m)



```c++
class Solution {
private:
    const int BASE = 100007;
    const int TIMES = 31;

public:
    int strStr(string haystack, string needle) {
        if (needle.empty())
        {
            return 0;
        }

        const int needle_size = needle.size();
        if (haystack.size() < needle_size)
        {
            return -1;
        }
        
        int power = 1;
        for (int i = 0; i < needle_size; i++)
        {
            power = power * TIMES % BASE;
        }

        int needle_code = 0;
        for (int i = 0; i < needle_size; i++)
        {
            needle_code = (needle_code * TIMES + needle[i]) % BASE;
        }

        int code = 0;
        for (int i = 0; i < haystack.size(); i++)
        {

            // add new
            code = (code * TIMES + haystack[i]) % BASE;
            if (i < needle_size - 1)
            {
                continue;
            }

            // delete first
            if (i >= needle_size)
            {
                code -= haystack[i - needle_size] * power % BASE;
                if (code < 0)
                    code += BASE;
            }

          	// 其实需要再验证一下
            if (code == needle_code)
            {
                return (i - needle_size + 1);
            }
        }

        return -1;
    }
};
```

