---
title: linux 搭建FTP服务器
date: 2021-06-30 10:42:35
tags:
- linux
categories:
- memo
---



#### 安装ftp和vsftpd

```bash
sudo apt-get install ftp
sudo apt-get install vsftpd
```





#### 启动服务

```bash
# 使用vsftpd软件，主要包括如下几个命令：

# 启动ftp
service vsftpd start

# 停止ftp
service vsftpd stop

# 重启ftp
service vsftpd restart
```





#### 配置vsftpd

打开配置文件

```bash
vi /etc/vsftpd.conf

# vi /etc/vsftpd/vsftpd.conf
```

按照需求修改为以下内容

```
listen=NO
listen_ipv6=YES
anonymous_enable=NO
local_enable=YES
write_enable=YES
dirmessage_enable=YES
use_localtime=YES
xferlog_enable=YES
connect_from_port_20=YES
secure_chroot_dir=/var/run/vsftpd/empty
pam_service_name=vsftpd
rsa_cert_file=/etc/ssl/certs/ssl-cert-snakeoil.pem
rsa_private_key_file=/etc/ssl/private/ssl-cert-snakeoil.key
ssl_enable=NO

# local root
local_root=/home/bot/dev/moma_app/scanner_images
```





#### 重启服务

```bash
service vsftpd restart
```





#### 登录测试

```bash
# 登录本地ftp
ftp 127.0.0.1

# 输入用户名和密码

# 查看当前路径是否设置的local root
pwd
```

