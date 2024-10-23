---
title: 使用vscode在docker容器里调试python代码
date: 2024-10-15 15:46:04
tags:
- debug
- vscode
- docker
- python
- coding
categories:
- [memo]
- [python]
- [docker]
keywords:
- vscode
- docker
- container
- debug
- 单步
- 调试
- python
description:
---

使用vscode，可以使用插件连接docker container，并基于容器运行调试代码。



### 在vscode内安装插件

- Dev Containers

  ![image-1](./use-vscode-to-debug-in-container/image-1.png#pic_left) 

- Docker
  ![image-2](./use-vscode-to-debug-in-container/image-2.png#pic_left) 



### 启动容器

```bash
# create container from image
# ...
# start container
docker start container_xxx
```



### 在vscode内连接容器

- 点击左下角的链接按钮

  ![image-3](./use-vscode-to-debug-in-container/image-3.png#pic_left) 


- 在弹出的选项中选择 **Attach to Running Container**，选择刚刚已经启动的容器 container_xxx

  ![image-4](./use-vscode-to-debug-in-container/image-4.png#pic_left) 



### 调试代码

在新打开的vscode窗口内打开对应的代码，添加断点，即可正常调试。