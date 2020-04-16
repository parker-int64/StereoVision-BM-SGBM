#ifndef __SOCKET_SERVER_H
#define __SOCKER_SERVER_H
#include <iostream>
#include <string.h>
#include <winsock.h>
#include <opencv2/opencv.hpp>

class socketSer {
    public:
        socketSer();
        ~socketSer();
        void socketInit();
        void socketRecv();
        void socketSend(char *sendData);
        void strToInt(char *pStr,double nIntData[],const char * pTok );
        
    private:
        SOCKET socketServer;
        sockaddr_in sin;
        int PORT;
        u_long ADDRESS;
        int length;
        char *recvData;
        
};
cv::FileStorage fs;
#endif // !__SOCKET_SERVER_H