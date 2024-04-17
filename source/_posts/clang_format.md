---
title: clang-format格式文件存档
date: 2019-09-04 13:52:31
tags:
- clang
- format
- c++
categories:
- memo
encrypt:
description:
---



常用 `.clang-format` `style` 文件存档



<!-- more -->

##### Google Style

```
BasedOnStyle: Google
IndentWidth: 2
ColumnLimit: 80
DerivePointerAlignment: false
PointerAlignment: Left
```



##### LLVM Style

```
BasedOnStyle:  LLVM
ColumnLimit:    80
AlignAfterOpenBracket: AlwaysBreak
AlignEscapedNewlines: Left
AllowShortFunctionsOnASingleLine: InlineOnly
AlwaysBreakTemplateDeclarations: Yes
BinPackArguments: false
BinPackParameters: false
BreakBeforeBraces: Allman
ConstructorInitializerAllOnOneLineOrOnePerLine: true
IncludeBlocks:   Preserve
IndentCaseLabels: true
NamespaceIndentation: All
PointerAlignment: Left
SortIncludes: false
TabWidth:        2
```

