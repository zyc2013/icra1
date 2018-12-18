//
#ifndef T_DT2019VISION_USART_H
#define T_DT2019VISION_USART_H
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h> //文件控制定义
#include <termios.h>//终端控制定义
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <mutex>
#include <vector>
#define DEVICE "/dev/ttyUSB0"
//#define DEBUG
#define  total_send_length 11
#define  receive_length 11
//#define S_TIMEOUT
using namespace std;
class Class_Usart
{
public:
    static int serial_fd;
    static unsigned int reserved_send_length;


    void Usart_Init();
    int Usart_Send(int Yaw,int Pitch,float Distance,bool Beat,bool NoObj);
    int Usart_Recv(vector<int>data);
    uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
    uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
    void Append_CRC16_Check_Sum(uint8_t  * pchMessage,uint32_t dwLength);



};

#endif //T_DT2019VISION_USART_H