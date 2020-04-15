#include "serial.h"
#include <thread>
serial::serial(){
    comName  = "COM5";
    BaudRate = CBR_115200;  // 波特率115200
    ByteSize = 8;           // 每个字节8位
    Parity  = NOPARITY;     // 无奇偶校验
    StopBits = TWOSTOPBITS; // 两个停止位
}

bool serial::openPort(){
    serial();
    hCom = CreateFileA(comName,GENERIC_READ | GENERIC_WRITE, // 存储方式读写
                        0,               //独占方式
                        NULL,
                        OPEN_EXISTING,  // 打开而不是创建
                        0,              //同步方式
                        NULL);
    if(hCom == (HANDLE)-1){
#ifdef DEBUG
        std::cout<< comName << "打开失败!"<<std::endl;
#endif // DEBUG
        return false;
    } else {
#ifdef DEBUG
        std::cout << comName << "打开成功!" << std::endl;
#endif // DEBUG
        return true;
    }
    SetupComm(hCom,1024,1024);
    COMMTIMEOUTS timeOuts;
    // 设置读超时
    timeOuts.ReadIntervalTimeout = 1000;
    timeOuts.ReadTotalTimeoutConstant = 5000;
    timeOuts.ReadTotalTimeoutMultiplier = 500;
    // 设置写超时
    timeOuts.WriteTotalTimeoutConstant = 500;
    timeOuts.WriteTotalTimeoutMultiplier = 2000;
    SetCommTimeouts(hCom,&timeOuts);    // 设置超时

    DCB dcb;
    GetCommState(hCom,&dcb);
    dcb.BaudRate = BaudRate;
    dcb.ByteSize = ByteSize;
    dcb.Parity   = Parity;
    dcb.StopBits = StopBits;
    SetCommState(hCom,&dcb);
    PurgeComm(hCom,PURGE_TXCLEAR|PURGE_RXCLEAR); // 清空缓冲区
}

bool serial::readPort(){
    readStatus = ReadFile(hCom,str,100,&wordCount,NULL);
    if(!readStatus){
#ifdef DEBUG
        std::cout << "读取串口信息失败!"<<std::endl;
#endif // DEBUG   
        return false;
    } else {
#ifdef DEBUG
        std::cout << "读取信息成功!" << std::endl;
#endif // DEBUG
        return true;
        } 

}

bool serial::writePort(char *outputBuffer){
    ClearCommError(hCom,&dwErrorFlags,&comStat);
    writeStatus = WriteFile(hCom,outputBuffer,dwByteWrite,&dwByteWrite,NULL);
    if(!writeStatus){
#ifdef DEBUG
        std::cout << "串口写入信息失败!"<<std::endl;
#endif // DEBUG
        return false;
    } else {
#ifdef DEBUG
    std::cout << "串口写入成功!";
    std::cout << " 写入数据："<< outputBuffer[0]<<outputBuffer[1]<<outputBuffer[2]<<outputBuffer[3]<< std::endl;
#endif // DEBUG
        return true;
        }
    PurgeComm(hCom, PURGE_TXABORT|PURGE_RXABORT|PURGE_TXCLEAR|PURGE_RXCLEAR);

    
}


serial::~serial(){
    CloseHandle(hCom);
#ifdef DEBUG
    std::cout << comName << "关闭成功!" << std::endl;
#endif // DEBUG
}

