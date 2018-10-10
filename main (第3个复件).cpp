
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
using namespace cv;
using namespace std;
struct contourboxs{
    Point2i center;//2-dimension class, include x,y axls.
    int height,width,area;
};
vector<contourboxs> Rectboxs;
#define pi 3.14159265358979323846
int fd;
bool send_new=0;//初始值不发送，当检测到激光点开启，发数
bool recv_new=0;//初始值不处理，当读到数再处理
char send_buf[7];
char rcv_buf[100]; 
int detect();
double t;//时间标志位
Rect temp_box;
void serial_thread();//串口函数
std::mutex serial_mutex;//某个锁
//串口发送函数
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
    VideoCapture cap(0);
    while(1)
    {
	t = (double)cvGetTickCount();                      //调用时钟测时间
        cap>>src;
        imshow("video",src);
        cvtColor(src, src_gray, COLOR_BGR2GRAY);
        threshold( src_gray, src_gray, 50, 250 , THRESH_BINARY );//二值化通常设置为50  255
	imshow("lvbo",src_gray);
	bitwise_not(src_gray,src_gray);//red=0,other=1
//        imwrite("picture1.jpg",drawing);
//        imwrite("picture2.jpg",src);
	int mark=detect();
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
        Mat drawing = Mat::zeros(src_gray.size(), CV_8UC3);
        //cout << "【筛选前总共轮廓个数为】：" << (int)contours.size() << endl;
        //ZHAO后的（开始）
    float Ang_X, Ang_Y, Ang_Z;
    float X, Y, Z;
    int i = 0;
    float A[][3] = { { 924.7125921651324, 0, 305.7203519641564 }, { 0, 924.0379484162191, 266.1595873144043 }, { 0, 0, 1 } };
    float B[] = { -0.4081105705665796, -0.08693170730276242, -0.003110104433910947, 0.003504029908282899, 0.1362767270209989 };
    Mat rvecs(3, 1, CV_32F), tvecs(3, 1, CV_32F), cameraMatrix(3, 3, CV_32F), distCoeffs(1, 5, CV_32F), R(3, 3, CV_32FC1);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            cameraMatrix.at<float>(i, j) = A[i][j];
            R.at<float>(i, j) = 0;
        }
    for (int i = 0; i < 5;i++)
        distCoeffs.at<float>(0, i) = B[i];
        vector<cv::Point3f> point3d;
            point3d.push_back(Point3f(-6.25,-6, 0));
            point3d.push_back(Point3f(-6.25,6, 0));
            point3d.push_back(Point3f(6.25,-6, 0));
            point3d.push_back(Point3f(6.25,6, 0));
            //ZHAO后的（结束）
        vector <vector<Point> >::iterator iter = contours.begin();
        for (; iter != contours.end();)//j zhi shi yi ge zeng liang
        {
            int j=0;
            double g_dConArea = contourArea(*iter);
            if (g_dConArea < 600)
            {
                iter = contours.erase(iter);
            }
            else 
            {
                Rect temp_box=boundingRect(contours[j]);//RECT class:创建一个矩形对象，矩形左上角的横坐标、纵坐标以及矩形的宽度、高度均为零
                //cout<<"width="<<temp_box.width<<"height="<<temp_box.height<<"x="<<temp_box.x<<"y="<<temp_box.y<<endl;
                if((temp_box.height<50)||(temp_box.width<50)||abs(temp_box.width-temp_box.height)>10)
                {
                    iter = contours.erase(iter);
                }
                else
                    //ZHAO注释掉的
             //       drawContours(src,contours ,j,color,2,8,hierarchy,0,Point());
                //drawContours(drawing,contours ,j,color,2,8,hierarchy,0,Point());

                {
 		++iter;
                j++;
            	}
        	}
		}
cout<<(int)contours.size()<<endl;
for (int j=0; j<contours.size();j++)//j zhi shi yi ge zeng liang
{
Rect temp_box=boundingRect(contours[j]);
cout<<"Rect.width="<<temp_box.width<<"Rect.height="<<temp_box.height<<endl;
		if((temp_box.height>60)&&(temp_box.width>60)&&abs(temp_box.width-temp_box.height)<10)
{
rectangle(src,temp_box,Scalar(0,255,0),2);
int x=temp_box.x+temp_box.width/2;
cout<<"x="<<x<<endl;
short TempInt1=x*100,TempInt2=Ang_Y*100;
			char TempChar1,TempChar2,TempChar3,TempChar4;
			//if (TempInt1<0)
                        //TempInt1=32768-TempInt1;
			send_buf[2]=0x01;
                        send_buf[3]=TempInt1>>8;
                        send_buf[4]=(TempInt1&0x00ff);
			//if (TempInt2<0)
                        send_buf[5]=TempInt2>>8;
                        send_buf[6]=(TempInt2&0x00ff);

break;
}
else
{
send_buf[0]=0xA5;
send_buf[1]=0xA5;	
send_buf[2]=0x00;
    send_buf[3]=0xff;
	send_buf[4]=0xff;
    send_buf[5]=0xff;
    send_buf[6]=0xff;
}
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
