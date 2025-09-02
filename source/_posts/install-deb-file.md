---
title: 在Linux中安装.deb文件
tags:
  - linux
  - ubuntu
  - debian
categories:
  - - memo
  - - linux
date: 2025-09-02 11:14:52
keywords:
description:
---



`.deb` 是 Debian、Ubuntu 及其衍生发行版（如 Linux Mint）使用的软件包格式。当无法从官方软件源获取软件时，我们常常需要手动下载并安装 `.deb` 文件。



---



## 方法一：使用 `apt` 命令

`apt` 是 Ubuntu 等系统的高级包管理工具。它最大的优点是**能够自动处理复杂的依赖关系**，是安装本地 `.deb` 文件的首选方法。

### 安装步骤

1.  **打开终端** (`Ctrl + Alt + T`)。
2.  **导航**到你所下载的 `.deb` 文件所在的目录。
    ```bash
    cd ~/Downloads  # 通常文件在"下载"目录中
    ```
3.  **执行安装命令**：
    ```bash
    sudo apt install ./PACKAGE_NAME.deb
    ```
    **务必注意**：命令中的 `./` 至关重要，它告诉 `apt` 从当前本地目录查找文件，而不是从远程软件库中搜索。

### 工作原理与示例

当你运行上述命令后，`apt` 会执行以下操作：
1.  分析该 `.deb` 包所需的依赖项。
2.  检查系统是否已满足这些依赖。
3.  **自动从配置的软件源下载并安装所有缺失的依赖**。
4.  最后安装该 `.deb` 包本身。



---



## 方法二：使用 `dpkg` 命令

`dpkg` 是 Debian 包管理系统的底层基础工具。它的缺点是**不会自动处理依赖关系**，通常需要与 `apt` 配合使用来完成安装。

### 安装步骤

1. **使用 `dpkg` 安装软件包**：

   ```
   sudo dpkg -i PACKAGE_NAME.deb
   ```

   - `-i` 选项代表 **i**nstall。

2. **修复依赖问题（通常必不可少）**：
   上一步操作很可能因缺少依赖而报错并中断。此时，运行以下命令来修复：

   ```
   sudo apt --fix-broken install
   ```

   或者

   ```
   sudo apt install -f  # `-f` 是 `--fix-broken` 的简写
   ```

   此命令会自动识别中断的安装，并下载安装所有必需的依赖包，从而完成整个安装过程。

### 工作原理与示例

`dpkg` 仅负责将软件包本身解压并复制文件到系统，不解决依赖。因此流程常是：

1. `dpkg -i` 尝试安装，但因依赖问题失败。

2. 使用 `apt -f install` 来修复中断的安装并满足依赖。

   

---



## 如何卸载已安装的 .deb 软件

卸载软件不需要原始的 `.deb` 文件，只需知道软件包名称即可。

### 使用 `apt` 进行卸载

1. **移除软件（保留配置文件）**：

   ```
   sudo apt remove PACKAGE_NAME
   ```

2. **彻底清除软件（同时删除配置文件）**：

   ```
   sudo apt purge PACKAGE_NAME  # 更彻底的清理
   ```



### 使用 `dpgk` 进行卸载

```bash
sudo dpkg -r PACKAGE_NAME
```



### 查找已安装的软件包名称

如果你不记得确切的包名，可以使用 `dpkg` 配合 `grep` 来查找。

```
dpkg -l | grep search-term

# 例如，查找所有和 chrome 相关的包：
dpkg -l | grep chrome
```

 
