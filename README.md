# StereoVision-BM-SGBM
OpenCV StereoBM &amp; StereoSGBM


## OpenCV 双目BM和SGBM算法
### 环境说明: 
操作系统：   Windows 10  
编译器：     MinGW-w64（GCC 8.1）  
OpenCV版本： 4.1  
CMake：     3.16  
请根据自己的OpenCV安装目录更改CMakeLists.txt的include_directories,link_directories和link_libraries ,换成对应的目录和文件。  
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
## 使用说明
本项目使用cvui界面进行参数调试:
[cvui](https://github.com/Dovyski/cvui/)  
[cvui使用说明](https://dovyski.github.io/cvui/)  
调试过程中，按键盘S键可切换StereoBM和StereoSGBM算法  
+ 按钮Save Parameter用以保存调试参数
+ 按钮Load Parameter用以从文件加载参数
+ 按钮Run Stereo Calibration暂时还没整合进来（项目是直接写入摄像头参数到变量中，没有生成yml文件）
+ Exit退出程序
