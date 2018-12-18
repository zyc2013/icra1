
//赵景玉修改版本
/*******************************************************/
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <math.h>
#include "serial.h"
#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
using namespace cv;
using namespace std;
struct contourboxs{
    Point2i center;//2-dimension class, include x,y axls.
    int height,width,area;
};
vector<contourboxs> Rectboxs;
#define pi 3.14159265358979323846
int fd,j,mark;
int device_;
bool send_new=0;//初始值不发送，当检测到激光点开启，发数
bool recv_new=0;//初始值不处理，当读到数再处理
char send_buf[7];
char rcv_buf[100]; 
int detect();
double t;//时间标志位
Rect temp_box,temp_box_result(1,1,1,1);
void serial_thread();//串口函数
std::mutex serial_mutex;//某个锁
//串口发送函数
bool LinenableExpose(bool flag){
    int ret;
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    if(flag)
        ctrl.value = V4L2_EXPOSURE_AUTO;
    else
        ctrl.value = V4L2_EXPOSURE_MANUAL;
    ret = ioctl(device_, VIDIOC_S_CTRL, &ctrl);
    if (ret < 0){
        std::cout<< "set camera expose failed!" << std::endl;
        return false;
    }
    return true;
}
bool LinsetExpose(int expose){
    int ret;
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = expose;
    ret = ioctl(device_, VIDIOC_S_CTRL, &ctrl);
    if (ret < 0){
        std::cout<< "set camera expose failed!" << std::endl;
        return false;
    }
    return true;
}

bool LinsetGain(int gain){
    int ret;
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_GAIN;
    ctrl.value = gain;
    ret = ioctl(device_, VIDIOC_S_CTRL, &ctrl);
    if (ret < 0){
        std::cout<< "set camera expose failed!" << std::endl;
        return false;
    }
    return true;
}

bool LinenableWriteBalance(bool flag){
    int ret;
    struct v4l2_control ctrl;
    ctrl.id=V4L2_CID_AUTO_WHITE_BALANCE;
    if(flag)
        ctrl.value=V4L2_WHITE_BALANCE_AUTO;
    else
        ctrl.value=V4L2_WHITE_BALANCE_MANUAL;
    ret=ioctl(device_,VIDIOC_S_CTRL,&ctrl);
    if(ret<0){
        std::cout<<"set camera writeblance failed!"<<std::endl;
        return false;
    }
    return true;
}
void serial_thread()
{
//extern int fd;
int err;                           //返回调用函数的状态      
while (1) //循环读取数据  
	{
		if(send_new)
		{
		serial_mutex.lock();
		//std::cout<<"send_new"<<send_new<<std::endl;
			int lens = UART0_Send(fd,send_buf,1);  
			send_new=0;//0
			if(lens > 0)  
				printf(" send %d data successful\n",lens);  
			else  
			{
				printf("send data failed!\n");
				exit (0);//tui chu zhu han shu  
			}
		}
			sleep(0.01);

			serial_mutex.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  
	}              
		//UART0_Close(fd);   
}  
Mat src,src_gray,src1;
int main()
{
int err;  //返回调用函数的状态  
//串口开启部分
if ((fd = UART0_Open(fd,"/dev/ttyUSB0"))==-1)
{
cout<<"/dev/ttyUSB error"<<endl;
return 0; //打开串口，返回文件描述符 
}
do
{  
		err = UART0_Init(fd,115200,0,8,1,'N');  //115200
		printf("Set Port Exactly!\n");  
	}while(FALSE == err || FALSE == fd);  
std::thread thread1(serial_thread);
thread1.detach();
LinenableExpose(false);
LinenableWriteBalance(true);
//LinsetExpose(20);
//`LinsetGain(0);
    VideoCapture cap(0);
    while(1)
    {
	t = (double)cvGetTickCount();                      //调用时钟测时间
        cap>>src;
        imshow("video",src);
        cvtColor(src, src_gray, COLOR_BGR2GRAY);
        threshold( src_gray, src_gray, 70, 250 , THRESH_BINARY );//二值化通常设置为50  255
	bitwise_not(src_gray,src_gray);//red=0,other=1
imshow("lvbo",src_gray);//        imwrite("picture1.jpg",drawing);
	     imwrite("picture2.jpg",src);
	mark=detect();
	if (mark!=0)
	{
	send_buf[3]=0xff;
	send_buf[4]=0xff;
	send_buf[5]=0xff;
	send_buf[6]=0xff;
	}
	send_new=1;
	t=(double)cvGetTickCount()-t;
        cout<<"used time is "<<(t/(cvGetTickFrequency()*1000))<<endl;
	//串口逻辑
	send_buf[0]=0xA5;
	send_buf[1]=0xA5;
	//send_buf[2]=0x00;
        if(waitKey(5)=='q')
        break;
    }
    return 0;
}
int detect()
{
        Mat threshold_output;
        vector <vector<Point> > contours;
        vector <Vec4i> hierarchy;
        Scalar color=Scalar(0,150,200);
        Canny(src_gray,src_gray,80,160,3);
        findContours(src_gray, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE,Point(0, 0));
        if(contours.empty())//without contours
        {
            cout<<"without signiture"<<endl;
            return 1;
        }
        for (int j=0; j<contours.size();j++)//j zhi shi yi ge zeng liang
        {
		temp_box=boundingRect(contours[j]);
if(temp_box.area()>temp_box_result.area())
{
	cout<<temp_box.area()<<endl;
	if(abs(temp_box.width-temp_box.height)<10)
            temp_box_result=temp_box;
cout<<temp_box_result.area()<<endl;
}
}
if (temp_box_result.width<50||temp_box_result.height<50||temp_box_result.area()<18000)
{
send_buf[0]=0xA5;
send_buf[1]=0xA5;	
send_buf[2]=0x00;
    send_buf[3]=0xff;
	send_buf[4]=0xff;
    send_buf[5]=0xff;
    send_buf[6]=0xff;
temp_box_result=Rect(1,1,1,1);
}
else
{
rectangle(src,temp_box_result,Scalar(0,255,0),2);
int x=temp_box_result.x+temp_box_result.width/2;
cout<<"x="<<x<<endl;
short TempInt1=x*100;short TempInt2=0*100;
			char TempChar1,TempChar2,TempChar3,TempChar4;
			//if (TempInt1<0)
                        //TempInt1=32768-TempInt1;
			send_buf[2]=0x01;
                        send_buf[3]=TempInt1>>8;
                        send_buf[4]=(TempInt1&0x00ff);
			//if (TempInt2<0)
                        send_buf[5]=TempInt2>>8;
                        send_buf[6]=(TempInt2&0x00ff);
temp_box_result=Rect(1,1,1,1);

}
/*if(1)
{
		//Rect temp_box=boundingRect(contours[0]);//RECT class:创建一个矩形对象，矩形左上角的横坐标、纵坐标以及矩形的宽度、高度均为零
                   	
		
                    //ZHAO后加的（开始）
                    vector<cv::Point2f> points2d;
                    points2d.push_back(Point2f(temp_box.x+temp_box.width,temp_box.y));
                    points2d.push_back(Point2f(temp_box.x+temp_box.width,temp_box.y+temp_box.height));
                    points2d.push_back(Point2f(temp_box.x,temp_box.y));
                    points2d.push_back(Point2f(temp_box.x,temp_box.y+temp_box.height));
                    solvePnP(point3d, points2d, cameraMatrix, distCoeffs, rvecs, tvecs);
                    Rodrigues(rvecs, R);
                    Ang_X = asin(R.at<double>(1, 0) / cos(asin(-R.at<double>(2, 0)))) / pi * 180;
                    Ang_Y = asin(-R.at<double>(2, 0)) / pi * 180;
                    Ang_Z = asin(R.at<double>(2, 1) / cos(asin(-R.at<double>(2, 0)))) / pi * 180;
                    X = R.at<double>(0, 0) *point3d[22].x + R.at<double>(0, 1)  * point3d[22].y + R.at<double>(0,2)  * point3d[22].z + tvecs.at<double>(0,0);
                    Y = R.at<double>(1, 0) *point3d[22].x + R.at<double>(1, 1)  * point3d[22].y + R.at<double>(1, 2)  * point3d[22].z + tvecs.at<double>(1, 0);
                    Z = R.at<double>(2, 0) *point3d[22].x + R.at<double>(2, 1)  * point3d[22].y + R.at<double>(2, 2)  * point3d[22].z + tvecs.at<double>(2, 0);
                    cout<<"X:"<<X<<" 偏转角度为"<<Ang_X<<endl;
                    cout<<"Y:"<<Y<<" 偏转角度为"<<Ang_Y<<endl;
                    cout<<"Z:"<<Z<<" 偏转角度为"<<Ang_Z<<endl;
                    //ZHAO后加的（结束）

			
                        //serOut.write(TempChar);
}*/
        //cout << "【筛选后总共轮廓个数为：" << (int)contours.size() << endl;

/*for(int i=0 ; i < (int)contours.size(); i++ )
                {

                }
                */
        imshow("处理后的图像",src);
return 0;
}
