---
title: Linux 解压缩rar包
date: 2021-09-26 09:57:35
tags:
- linux
- uncompress
- rar
categories:
- [memo]
- [linux]
encrypt:
description:
---



#### 下载rar软件安装包

1. 直接从 [rarlab]( https://www.rarlab.com/download.htm) 下载安装包

2. 通过命令行下载

   ```bash
   wget https://www.rarlab.com/rar/rarlinux-x64-6.0.1.tar.gz  # 64位
   wget https://www.rarlab.com/rar/rarlinux-3.8.0.tar.gz	   # 32位
   ```



#### 安装


```bash
tar zxvf rarlinux-x64-6.0.1.tar.gz
cd rar
sudo make
sudo make install
```



#### 解压缩

```bash
rar x xxx.rar  # Extract files with full path
rar e xxx.rar  # Extract files without archived paths
```



<!-- more -->



#### 将xxx目录压缩为xxx.rar

```bash
rar a xxx.rar xxx
```


