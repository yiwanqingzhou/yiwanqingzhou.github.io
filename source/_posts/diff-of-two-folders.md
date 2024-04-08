---
title: linux 比较两个目录的区别
date: 2024-04-01 19:27:14
tags:
- linux
categories:
- memo
keywords:
- linux
- diff
description:
---

### 查看两个目录的结构

```bash
$ tree test_a/ test_b/
```

```bash
test_a/
└── test.txt
test_b/
├── test_b.txt
└── test.txt
```

### 使用 diff 逐行比较

```bash
$ diff -aur test_a/ test_b/
```

```bash
Only in test_b/: test_b.txt
diff -aur test_a/test.txt test_b/test.txt
--- test_a/test.txt	2024-04-01 19:29:35.024777745 +0800
+++ test_b/test.txt	2024-04-01 19:36:22.759553212 +0800
@@ -1 +1 @@
-test_a
+test_b
```

上面结果可以看到两个文件夹里相同文件名 `test.txt` 里的细节区别，而对于 test_b 文件夹里的 test_b.txt，只显示 `Only in test_b/: test_b.txt`

使用 `-N` 则可以同时打印新文件的内容

```bash
$ diff -Naur test_a/ test_b/
```

```bash
diff -Naur test_a/test_b.txt test_b/test_b.txt
--- test_a/test_b.txt	1970-01-01 08:00:00.000000000 +0800
+++ test_b/test_b.txt	2024-04-01 19:29:48.008986470 +0800
@@ -0,0 +1 @@
+extra_test_b
diff -Naur test_a/test.txt test_b/test.txt
--- test_a/test.txt	2024-04-01 19:29:35.024777745 +0800
+++ test_b/test.txt	2024-04-01 19:36:22.759553212 +0800
@@ -1 +1 @@
-test_a
+test_b
```

### 只显示有区别的文件名

使用 `-q` 可以快速显示有区别的文件名而非具体内容

```bash
$ diff -rq test_a/ test_b/
```

```bash
Only in test_b/: test_b.txt
Files test_a/test.txt and test_b/test.txt differ
```

加上 `-N`

```bash
$ diff -Naurq test_a/ test_b/
```

```bash
Files test_a/test_b.txt and test_b/test_b.txt differ
Files test_a/test.txt and test_b/test.txt differ
```
