#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <math.h>
#include "usart.h"
#include <thread>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
using namespace std;
using namespace cv;
class Class_Usart usart;
vector<int> data;
void serial_thread();//串口函数
std::mutex serial_mutex;//某个锁
void serial_thread()
{
//extern int fd;
int err;                           //返回调用函数的状态      
while (1) //循环读取数据  
	{
		//if(send_new)
		//{
		serial_mutex.lock();
		//std::cout<<"send_new"<<send_new<<std::endl;
			if (usart.Usart_Recv(data)==0)
		{
			cout<<data.size()<<endl;
		}
		sleep(0.01);
		serial_mutex.unlock();
       		std::this_thread::sleep_for(std::chrono::milliseconds(10));  
	//}              
		//UART0_Close(fd);   
	}  
}
int main(void)
{
	
	usart.Usart_Init();
	std::thread thread1(serial_thread);
	thread1.detach();
	while(1)
	{
	//if (usart.Usart_Recv(data)==0)
	//{
	//	cout<<data.size()<<endl;
	//}
	//cout<<"1"<<endl;
	usart.Usart_Send(1,1,100,1,1);
	if (waitKey(10)==27)
	{
		return 1;
	}
}
return 0;
}

