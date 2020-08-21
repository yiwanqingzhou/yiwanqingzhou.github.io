---
title: Debian最小化按钮
date: 2019-09-04 11:29:34
tags:

---





安装tweak和dconf

```
sudo apt install gnome-tweak-tool
sudo apt install dconf-editor 
dconf-editor
```

<!-- more -->



进入路径: `org/gnome/desktop/wm/preferences/button-layout`



{% asset_img test.png Debian最小化按钮 %}



取消选项 Use default value

在 Custom value 填入 `appmemu:close,minimize,maximize,close`