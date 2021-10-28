---
title: Linux 代码自动格式化
date: 2021-09-17 17:59:23
tags:
- linux
categories:
- memo
encrypt:
description:
---



#### 使用`clang`对C++进行格式化

- 安装`clang-format-7`

  ```bash
  sudo apt-get install -y clang-format-7
  ```

- 运行脚本

  ```bash
  find . -regextype egrep -regex ".*\.(c|cc|h|hh)$" -not -path '*/install/*' \
    -not -path '*/build/*' -not -path '*/log/*' -not -path '*/deps/*'| xargs clang-format-7 -i
  ```

  



#### 使用`black`对python进行格式化

- 安装`black`

  ```bash
  pip3 install black==20.8b1
  ```

- 运行

  ```
  black .
  ```

  