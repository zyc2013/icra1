#ifndef __HH__
#define __HH__
#include <stdio.h>
#include <stdlib.h>     /*标准函数库定义*/  
#include <unistd.h>     /*Unix 标准函数定义*/  
#include <sys/types.h>   
#include <sys/stat.h>     
#include <fcntl.h>      /*文件控制定义*/  
#include <termios.h>    /*PPSIX 终端控制定义*/  
#include <errno.h>      /*错误号定义*/  
#include <string.h>  
#include <iostream>
#include <thread>
#include <mutex>


int UART0_Open(int fd,char* port);
void UART0_Close(int fd);
int UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity);
int UART0_Send(int fd, char *send_buf,int data_len);//串口号，发送内容，字符长度
  int UART0_Recv(int fd, char *rcv_buf,int data_len);
//宏定义
#ifndef def
#define def
#define FALSE  -1  
#define TRUE   0  
 
#endif
#endif

