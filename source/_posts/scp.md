---
title: Linux 使用SCP远程拷贝文件
date: 2021-10-22 11:15:01
tags:
- linux
- scp
categories:
- memo
encrypt:
---



 使用scp(secure copy)命令可以实现本地服务器和远程服务器之间的文件传输复制



<!-- more -->



#### 从本地复制到远程

```bash
scp test.txt bot@10.0.9.211:/home/bot/test_folder
scp test.txt bot@10.0.9.211:/home/bot/test_folder/test.txt
scp test.txt 10.0.9.211:/home/bot/test_folder
scp test.txt 10.0.9.211:/home/bot/test_folder/test.txt
```



#### 从远程复制到本地

```bash
scp bot@10.0.9.211:/home/bot/test_folder text.txt
scp bot@10.0.9.211:/home/bot/test_folder/test.txt test.txt
scp 10.0.9.211:/home/bot/test_folder test.txt
scp 10.0.9.211:/home/bot/test_folder/test.txt test.txt
```



#### 传输整个目录

```bash
scp -r /test_folder bot@10.0.9.211:/home/bot/test_folder
```



#### 其他参数用法

```bash
scp --help

usage: scp [-12346BCpqrv] [-c cipher] [-F ssh_config] [-i identity_file]
           [-l limit] [-o ssh_option] [-P port] [-S program]
           [[user@]host1:]file1 ... [[user@]host2:]file2

```

