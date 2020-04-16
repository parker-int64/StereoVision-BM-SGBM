#ifndef __SERIAL_H
#define __SERIAL_H
#include <iostream>
#include <windows.h>
class serial{
    public:
        serial();
        bool openPort();
        bool writePort(char *outputBuffer);
        bool readPort();
        ~serial();
    private:
        HANDLE hCom;            // 全局变量，串口句柄
        LPCSTR comName;     // 串口名
        DWORD fParity;          // 指定奇偶校验使能
        BYTE ByteSize;          // 通信字节位数，4—8
        BYTE Parity;            // 指定奇偶校验方法
        BYTE StopBits;          // 指定停止位的位数
        DWORD BaudRate;         // 波特率
        /*读取串口信息*/
        char str[100];
        DWORD wordCount;
        bool readStatus;
        /*信息写入串口*/
        DWORD dwByteWrite;
        COMSTAT comStat;
        DWORD dwErrorFlags;
        bool writeStatus;
};


#endif // !__SERIAL_H

