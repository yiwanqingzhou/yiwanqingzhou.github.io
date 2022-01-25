---
title: 使用clang和black 对C++、Python代码自动格式化
date: 2021-09-17 17:59:23
tags:
- linux
- format
- clang
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

- 编写脚本 `clang_format.bash`

  ```bash
  find . -regextype egrep -regex ".*\.(c|cc|h|hh)$" -not -path '*/install/*' \
    -not -path '*/build/*' -not -path '*/log/*' -not -path '*/deps/*'| xargs clang-format-7 -i
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

  