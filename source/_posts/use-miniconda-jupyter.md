---
title: 通过miniconda安装使用jupyter
date: 2024-03-26 16:55:24
tags:
- linux
- python
categories:
- memo
keywords:
- linux
- python
- miniconda
- jupyter
description:
---

## 安装 `miniconda`

从 https://docs.anaconda.com/free/miniconda/ 下载对应版本的安装脚本

```bash
bash xx.sh
```

根据提示安装完成后，重新打开terminal窗口

## 通过 `miniconda` 创建单独的环境且安装 `jupyter` 等需要的包

```bash
conda env list
conda create -n environment_name python numpy pandas jupyter
```

```bash
conda deactivate # exit current environment
conda activate environment_name # activate the specific environment

# can still install other packages
conda install matplotlib
```

## 运行 `jupyter`

```bash
jupyter notebook
```
