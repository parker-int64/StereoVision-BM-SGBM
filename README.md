# StereoVision-BM-SGBM
OpenCV StereoBM &amp; StereoSGBM
# 第二版，分辨率下降一半
环境没有变化  
在StereoBM算法下帧率可以达到40~50，StereoSGBM下帧率能达到20帧。  
就匹配结果来看，StereoSGBM得出的世界坐标更为准确，但是StereoBM的运行速度更快。  
这一版加上了串口传输，默认COM5为通信口，可根据需求改。    
![运行图](https://raw.githubusercontent.com/parker-int64/StereoVision-BM-SGBM/stereoMatch-0.2.0/data/Debug_running.png)

## 运行方式
```SHELL
rm bin/*
rm -r build/*
cd build && cmake .. -DCMKAE_BUILD_TYPE=Debug
cd ../bin
./stereoMatch
```
选用**release**来编译，设置`-DCMAKE_BUILD_TYPE=Release`，帧数会略微提高。
