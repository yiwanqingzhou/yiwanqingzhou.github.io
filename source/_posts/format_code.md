---
title: 使用clang和black 对C++、Python代码自动格式化
date: 2021-09-17 17:59:23
tags:
- linux
- format
- clang
categories:
- [memo]
- [linux]
encrypt:
description:
---



#### 使用`clang`对C++进行格式化

- 安装`clang-format`

  ```bash
  sudo apt-get install -y clang-format
  ```

- 简单使用

  ```bash
  clang-format -i <file_to_format>
  # clang-format -i test.cc
  ```

- 指定 `style`

  ```bash
  clang-format -i --stlye="{key: value, ...}"
  # clang-format -i --stlye="{BasedOnStyle: Google, IndentWidth: 2}"
  ```

- 指定 [style 文件](./clang_format.html)

  ```bash
  clang-format -i --stlye=file:<style_file>
  # clang-format -i --style=file:.clang-format
  ```

- 编写脚本 `clang_format.bash` 对文件夹下所有c++代码格式化

  ```bash
  find . -regextype egrep -regex ".*\.(c|cc|h|hh)$" | xargs clang-format -i
  
  # skip some paths
  # find . -regextype egrep -regex ".*\.(c|cc|h|hh)$" -not -path '*/install/*' -not -path '*/build/*' -not -path '*/log/*' -not -path '*/deps/*'| xargs clang-format -i
  ```

- 在需要格式化的路径下运行脚本

  ```bash
  ./clang_format.bash
  ```

  



#### 使用`black`对python进行格式化

- 安装`black`

  ```bash
  pip3 install black==20.8b1
  ```

- 在需要格式化的路径下运行

  ```
  black .
  ```

  