# Wuhu_Coordinator

此程序为 山魂四队 参加 第十七届全国大学生智能汽车竞赛群体智能挑战赛协同搬运组 比赛时，“边云结合”的“云”端程序，比赛时运行在电脑上。

技术方案参见博客文章：[协同搬运组上位机方案分享 | lookas](https://18kas.com/wuhu-coordinator)

## 环境需求

程序运行在 Windows 系统，需要安装有 Microsoft Visual Studio Tools (cl.exe)，Windows SDK (windows.h)，OpenCV，CMake，Git Bash

## 连接网络

比赛时使用一个手机打开热点，将设备们连接到网络上：

- 一台华为 nova 7 SE 5G 手机，ip 为 192.168.43.52，提供热点
- 一台红米 K30 Ultra 手机，ip 为 192.168.43.96
- 一辆车模（红车，左面(1,1)点处发车），ip 为 192.168.43.130
- 一辆车模（蓝车，右面(11,1)点处发车），ip 为 192.168.43.120

华为手机使用有线 ADB 直接连接到电脑上。

红米手机使用无线 ADB 连接到电脑上：

```
adb tcpip 5555
adb connect 192.168.43.96:5555
adb devices
```

两辆小车使用逐飞提供的 WiFi 模块通过热点连接到电脑上。

## 编译项目

```powershell
cd build
cmake ..
cmake --build .
```

## 运行

在摄像头架设完毕后，需要先对图像进行逆透视标定：

- `recv` 在两台手机上拍照，并且将图像传回电脑
- `calc` 将传回的图片进行标定，找到边界的4个角点（两个手机各管半场的图片，分别为 (3.4m,5m) 和 (3.6m,5m)）

在真正发车时，需要重新采集图片，进行逆透视变化，找到白色图像点，并且将点数据发回小车：

- `recv` 在两台手机上拍照，并且将图像传回电脑
- `handle` 将传回的图片进行处理，找到有图片的点
- `send` 将点数据传回小车

## 许可

[MIT](http://opensource.org/licenses/MIT)

Copyright (c) 2022, xiaoxi & lookas