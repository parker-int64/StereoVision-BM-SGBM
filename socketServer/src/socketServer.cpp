#include "socketServer.h"


socketSer::socketSer(){
    PORT    = 22500;
    ADDRESS = INADDR_ANY;
}

void socketSer::socketInit(){
    WORD socketVersion = MAKEWORD(2,2); 
    WSADATA wsaData;  
    if(WSAStartup(socketVersion, &wsaData) != 0) return ;
    socketServer = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    sockaddr_in sin;
    sin.sin_family = AF_INET;
    sin.sin_port   = htons(PORT);
    sin.sin_addr.S_un.S_addr = ADDRESS;
    length = sizeof(sin);
    int bindStatus = bind(socketServer, (sockaddr *)&sin, sizeof(sin));
    if(bindStatus ==SOCKET_ERROR){
        std::cout << "绑定套接字错误，关闭连接!" << std::endl;
        closesocket(socketServer);
    }
    
    
}

void socketSer::socketRecv(){
    char recvData[100];
    double temp[2] = {0.,0.};
    int readStatus = recvfrom(socketServer, recvData, 255, 0,(sockaddr *)&sin,&length);
    if(readStatus > 0){
        recvData[readStatus] = 0x00;
        std::cout << "UDP数据包接收自"<< inet_ntoa(sin.sin_addr) << "，内容: "<< recvData << "," ;
        strToInt(recvData,temp,","); 
    }
    cv::Point2d coord = cv::Point2d(temp[0],temp[1]);
    
    fs << "POINT_2D" << coord;
    fs.release();
    std::cout << "二维点: " << coord << std::endl;
}

void socketSer::socketSend(char *sendData){
    int sendStatus = sendto(socketServer,sendData,strlen(sendData), 0, (sockaddr *)&sin,length);
    if(sendStatus > 0){
        std::cout << "UDP数据包发送成功，内容："<< sendData << std::endl;
    }
}


void socketSer::strToInt( char *pStr,double nIntData[],const char * pTok )
    {

        char *pStrPos = NULL;
        char *pNextToken = NULL;
        int nPos = 0;

        pStrPos = strtok_s(pStr,pTok,&pNextToken);

        while (pStrPos) 
        {
            nIntData[nPos] = atof(pStrPos);
            nPos ++;
            pStrPos = strtok_s(NULL,pTok,&pNextToken);
        } 

    }

socketSer::~socketSer(){
    closesocket(socketServer);
    WSACleanup();
    fs.release(); 
}

int main(){
    fs.open("../data/coord2D.yml",cv::FileStorage::WRITE | cv::FileStorage::APPEND);
    if(!fs.isOpened())exit(0);
    fs.writeComment("This coordinates is auto generated,do not edit!");
    fs << "Default" << cv::Point(0,0);
    socketSer socketSer;
    socketSer.socketInit();
    while (true){
        socketSer.socketRecv();
    }
    return 0;
}
