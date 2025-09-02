---
title: VSCode中让TXT文件自动换行的方法
date: 2025-08-25
tags:

---



在使用 VSCode 编辑 `.txt` 文件或其他文件时，有时内容过长会超出编辑器宽度，影响阅读体验。

---



## 方法1：临时开启自动换行

1. 打开 `.txt` 文件。
2. 在菜单栏点击 **查看(View)** → **切换自动换行(Toggle Word Wrap)**。
3. 或者使用快捷键：

- Windows/Linux: `Alt + Z`
- macOS: `Option + Z`

这样当前打开的文件就会自动换行显示。再次使用相同操作可以关闭自动换行。

---



## 方法2：针对所有文件默认开启自动换行

如果你希望所有文件默认自动换行，可以修改 VSCode 的设置：

1. 打开设置（快捷键 `Ctrl + ,` 或 `Cmd + ,`）。
2. 搜索 `word wrap`。
3. 找到 **Editor: Word Wrap**，将其设置为 `on`。

---

## 方法3：针对所有文本文件默认开启自动换行

如果只想针对 `.txt` 文件生效，可以在 `settings.json` 中添加如下配置：

```json
"[plaintext]": {
    "editor.wordWrap": "on"
}
