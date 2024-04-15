---
title: linux 使用sed全局搜索关键字替换
date: 2024-04-15 11:46:02
tags:
- linux
categories:
- [memo]
- [linux]
keywords:
- linux
- 查找
- 搜索
- 替换
- sed
description:
---



### 流编辑器 Sed

> [`sed` 是一种流编辑器]((https://wangchujiang.com/linux-command/c/sed.html))，它是文本处理中非常重要的工具，能够完美的配合正则表达式使用，功能不同凡响。处理时，把当前处理的行存储在临时缓冲区中，称为“模式空间”（pattern space），接着用 `sed` 命令处理缓冲区中的内容，处理完成后，把缓冲区的内容送往屏幕。接着处理下一行，这样不断重复，直到文件末尾。文件内容并没有 改变，除非你使用重定向存储输出。`Sed` 主要用来自动编辑一个或多个文件；简化对文件的反复操作；编写转换程序等。

#### 命令格式

```bash
sed [options] 'command' file(s)
sed [options] -f scriptfile file(s)  # 以选项中指定的script文件来处理输入的文本文件
```

#### 常用命令

- `s` : 替换
- `p` : 打印
- `i` : 插入
- `d` : 删除

#### 常用选项

- `-n`：默认情况下，sed会打印所有处理过的行，使用-n选项后，只打印经过sed特殊处理的行。
- `-i`：直接修改文件内容
- `-e script`：向sed添加多条命令来执行

#### 一些例子

```bash
# echo 直接操作 -> 将(第一个) a 替换成 aaa
$ echo "I am a test file" | sed 's/a/aaa/'
I aaam a test file
```

```bash
# 创建test_file.txt
$ touch test_file.txt
$ echo "I am a test file" > test_file.txt
$ cat test_file.txt
I am a test file

# 对test_file.txt的内容 -> 将(第一个) a 替换成 aaa
$ sed 's/a/aaa/' test_file.txt
I aaam a test file

# 加g可以全局替换
$ sed 's/a/aaa/g' test_file.txt 
I aaam aaa test file

# 以上都只是打印在terminal，没有对文件进行实际修改
# 加上 -i 才会真正修改文件
$ cat test_file.txt 
I am a test file

$ sed 's/a/aaa/g' -i test_file.txt 
$ cat test_file.txt 
I aaam aaa test file
```

#### 使用正则式

- `^` : 匹配行开始，如：`/^sed/` 匹配所有以 `sed` 开头的行
- `$` : 匹配行结束，如：`/sed$/` 匹配所有以 `sed` 结尾的行
- `.`: 匹配一个非换行符的任意字符，如：`/s.d/` 匹配 `s` 后接一个任意字符，最后是 `d` 
- `*` : 将 `*` 前面的正则表达式匹配重复 `0-N` 次
- `\+` : 同`*`, 但匹配重复 `1` 或 `N` 次
- `\?` : 同`*`, 但匹配重复 `0` 或 `1` 次
- `\{i\}` : 同`*`, 但匹配重复 `i` 次
- `\{i,j\}` : 同`*`, 但匹配重复 `i-j` 次
- `\{i,\}`: 同`*`, 但匹配重复至少 `i` 次
- `[]` : 匹配一个指定范围内的字符，如`/[sS]ed/` 匹配 `sed` 和 `Sed` 
- `[^]` : 匹配一个不在指定范围内的字符，如：`/[^A-RT-Z]ed/` 匹配不包含 `A-R` 和 `T-Z` 的一个字母开头，紧跟 `ed` 的行

- `\(regexp\)` : 将 `regexp` 看作一个整体，标记一个子表达式的开始和结束位置，用于后向引用，与 `\digit` 配合使用
- `\digit` ： 匹配正则表达式中定义的第 `digit` 个子表达式，`digit` 为 `1-9` 的数字
- `&` : 匹配的整个字符串

```bash
$ echo "I am a test file" > test_file.txt
$ cat test_file.txt
I am a test file

$ sed -e 's/[a-z]\+/[&]/g' test_file.txt
I [am] [a] [test] [file]
sed -e 's/[a-zA-Z]\+/[&]/g' test_file.txt
[I] [am] [a] [test] [file]

```

#### 处理大量文件

- 对多个文件进行处理：

```bash
sed 's/old-text/new-text/g' file1.txt file2.txt file3.txt
```

- 对大量文件进行处理：

```bash
sed -n 's/old-text/new-text/g' *.txt
sed -i 's/old-text/new-text/g' *.txt
```

- 结合 `find` 查找文件进行处理：

```bash
# 查找所有 `.hh` 和 `.cc` 文件
find . -iname "*.hh" -o -iname "*.cc" 

# 对find搜索到的文件进行替换 (可以用grep高亮)
find . -iname "*.hh" -o -iname "*.cc" | xargs sed -n 's/old-text/new-text/g' | grep new-text
find . -iname "*.hh" -o -iname "*.cc" | xargs sed -i 's/old-text/new-text/g' 
```
