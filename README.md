# 较为实用的第二版已经发布，branch下选择stereoMatch-0.2.0以查看
# StereoVision-BM-SGBM
## OpenCV 双目BM和SGBM算法
### 环境说明: 
操作系统：   Windows 10  
编译器：     MinGW-w64（GCC 8.1）  
OpenCV版本： 4.1  
CMake：     3.16  
请根据自己的OpenCV安装目录更改`CMakeLists.txt`的`include_directories`,`link_directories`和`link_libraries` ,换成对应的目录和文件。  
（由于是在windows上开发的，所以没有用pkg来寻找库）

## 运行方式：
```SHELL
rm -r build/*
cd build
cmake ..
make -j4
./main
```
根据平台不同，命令可能会有所不同，如在Windows下用MinGW-w64（Powershell）：
```SHELL
rm -r build/*
cd build 
cmake .. -G "MinGW Makefiles"
mingw-make -j4
./main.exe
```
## 运行结果
![运行图](https://raw.githubusercontent.com/parker-int64/StereoVision-BM-SGBM/master/data/running.png)
+ 双目分辨率是1280*480，**图中的分辨率有误，应该为每秒10帧左右**
+ 最后运算完成后得到深度图时间在**0.19~0.21**秒之间，**也即FPS大概为5**。
+ 降低分辨率可以提升运算速度
## 使用说明
本项目使用cvui界面进行参数调试:  
[cvui](https://github.com/Dovyski/cvui/)  
[cvui使用说明](https://dovyski.github.io/cvui/)  
调试过程中，按键盘S键可切换StereoBM和StereoSGBM算法  
+ 按钮`Save Parameter`用以保存调试参数
+ 按钮`Load Parameter`用以从文件加载参数
+ 按钮`Run Stereo Calibration`暂时还没整合进来（项目是直接写入摄像头参数到变量中，没有生成yml文件）
+ 按钮`Exit`退出程序
+ 紫色变量是对生成的深度图质量好坏影响很大的变量
+ 终端默认生成的三维坐标是像素重点\[320,240\]对应的世界坐标
