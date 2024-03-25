---
title: Ubuntu环境下安装使用hexo
date: 2024-03-25 17:48:00
tags: 
- linux
categories:
- memo
keywords: 
- hexo
- linux
- ubuntu
description:
---



## Install Git

```bash
sudo apt install git
git config --global user.name "Your Name"
git config --global user.email "youremail@domain.com"
```



## Configure Firewall

```bash
sudo apt install ufw
sudo ufw allow "OpenSSH"
sudo ufw enable
sudo ufw allow http
sudo ufw allow https
```

Check the firewall status.

```bash
sudo ufw status
Status: active

To                         Action      From
-                          --          --
OpenSSH                    ALLOW       Anywhere                  
4000                       ALLOW       Anywhere                  
80/tcp                     ALLOW       Anywhere                  
443                        ALLOW       Anywhere                  
OpenSSH (v6)               ALLOW       Anywhere (v6)             
4000 (v6)                  ALLOW       Anywhere (v6)             
80/tcp (v6)                ALLOW       Anywhere (v6)             
443 (v6)                   ALLOW       Anywhere (v6)
```



## Install `nodejs` via `nvm`

```bash
# installs NVM (Node Version Manager)
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash
# download and install Node.js
nvm install 20
# verifies the right Node.js version is in the environment
node -v # should print `v20.11.1`
# verifies the right NPM version is in the environment
npm -v # should print `10.2.4`
```



## Install `hexo`

```bash
sudo npm install hexo-cli -g
```



## Init the hexo blog

```bash
mkdir blog  # create the blog folder
cd blog
hexo init
```

