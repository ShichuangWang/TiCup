#include <iostream>
#include "opencv2/opencv.hpp"
#include "wiringPi.h"
#include "wiringSerial.h"

using namespace std;
using namespace cv;

VideoCapture capture(0,CAP_V4L2);
Mat frame,lab,red,green,r_ch,g_ch,binary,binary2,tmp,tmp2,black;
Mat r_binary,g_binary;
Scalar red_l=Scalar(80,140,80);
Scalar red_h=Scalar(190,250,160);

Scalar green_l=Scalar(80,80,80);
Scalar green_h=Scalar(160,160,160);

Scalar black_l=Scalar(0,0,0);
Scalar black_h=Scalar(15,255,255);

vector<Mat> channels;
Point getPos(Mat &img);
void init();
Mat splitRed(Mat &input);
Mat splitGreen(Mat &input);
Mat splitBlcak(Mat &input);
Point getPos(Mat &img,Mat&src_img,bool drawOut);
void taskHandler();
void taskConfig(int number);
Point p1,p2,p3;
Point rp,gp;
Mat H;

int fd;

int cnt=0;
bool save=false;
bool calib_flag=false;
Point2f src_point[4];
Point2f dst_point[4];
Point corn_point[4];
int x_err1,x_err2,x_err3;
int y_err1,y_err2,y_err3;
Mat dst;
int opt=0;
int task_num=2;
int point_get_cnt=0;
vector<Point> route;
Mat element;


int main() {

  init();
  x_err1=0;x_err2=0;x_err3=0;
  y_err1=0;y_err2=0;y_err3=0;
  taskConfig(2);

  
  while (true)
  {
    // cout<<"CAP_PROP_EXPOSURE:"<<capture.get(CAP_PROP_EXPOSURE)<<endl;
    capture>>frame;
    if(!frame.empty())
    {
      if(calib_flag)
      {
        #if 0
        warpPerspective(frame,dst,H,Size(500,500));
        circle(dst,Point(250,250),3,Scalar(255,0,0),-1);
        r_binary=splitRed(dst);
        g_binary=splitGreen(dst);
        rp= getPos(r_binary,dst,true);
        gp= getPos(g_binary,dst,true);
        // if(!save)
        // {
        //   imwrite("/home/orangepi/Desktop/1.png",dst);
        //   save=false;
        // }
        
        // cout<<dst.size()<<endl;
        x_err3=rp.x-250;y_err3=rp.y-250;
        int avg_x=(x_err1+x_err2+x_err3)/3;
        int avg_y=(y_err1+y_err2+y_err3)/3;
        serialPrintf(fd,"X%dY%dF",avg_x,avg_y);
        x_err1=x_err2;y_err1=y_err2;
        x_err2=x_err3; y_err2=y_err3;
        printf("X%dY%dF\n",avg_x,avg_y);
        #endif
        int64 t1=cv::getTickCount();
        taskHandler();
        imshow("dst",dst);
        int64 t2=cv::getTickCount();
        //cout<<"tim:"<<(t2-t1)/cv::getTickFrequency()<<endl;
        
      }
      else
      {
        #if 0
        cvtColor(frame,lab,COLOR_BGR2Lab);
        inRange(lab,black_l,black_h,black);

        split(frame,channels);//通道分离
        r_ch=channels.at(2);//r通道
        g_ch=channels.at(1);//g通道

        tmp=max((r_ch-g_ch),0)*3;//消除负值，提高对比度
        tmp2=max(-(r_ch-g_ch),0)*3;//消除负值，提高对比度

        threshold(tmp,binary,128,255,THRESH_BINARY);//二值化
        threshold(tmp2,binary2,128,255,THRESH_BINARY);//二值化

        dilate(binary,binary,element);//膨胀
        dilate(binary2,binary2,element);//膨胀
        #endif
        binary=splitRed(frame);
        binary2=splitGreen(frame);
        p1= getPos(binary);
        p2= getPos(binary2);
        imshow("frame",frame);
        imshow("binary",binary);

        imshow("binary2",binary2);
      }

      opt=waitKey(1);
      if(opt==32)
      {
        src_point[cnt]=p1;
        cnt++;
        cout<<"cnt:"<<cnt<<endl;
        if(cnt==4)
        {
          cout<<"get all point"<<endl;
          H= getPerspectiveTransform(src_point,dst_point);
          calib_flag= true;
        }
      }
      if(opt==65)
      {
        taskConfig(2);
      }
    }
  }

  return 0;
}

Mat splitRed(Mat &input)
{
  vector<Mat> tmp_channels;
  Mat tmp_r_ch,tmp_g_ch;
  Mat tmp_res,tmp_binary;
  split(input,tmp_channels);
  tmp_r_ch=tmp_channels.at(2);
  tmp_g_ch=tmp_channels.at(1);
  tmp_res=max((tmp_r_ch-tmp_g_ch),0)*3;
  threshold(tmp_res,tmp_binary,128,255,THRESH_BINARY);//二值化
  dilate(tmp_binary,tmp_binary,element);//膨胀
  return tmp_binary;
}

Mat splitGreen(Mat &input)
{
  vector<Mat> tmp_channels;
  Mat tmp_r_ch,tmp_g_ch;
  Mat tmp_res,tmp_binary;
  split(input,tmp_channels);
  tmp_r_ch=tmp_channels.at(2);
  tmp_g_ch=tmp_channels.at(1);
  tmp_res=max(-(tmp_r_ch-tmp_g_ch),0)*3;
  threshold(tmp_res,tmp_binary,128,255,THRESH_BINARY);//二值化
  dilate(tmp_binary,tmp_binary,element);//膨胀
  return tmp_binary;
}

Mat splitBlcak(Mat &input)
{
  Mat tmp_lab,tmp_black;
  cvtColor(input,tmp_lab,COLOR_BGR2Lab);
  inRange(tmp_lab,black_l,black_h,tmp_black);
  medianBlur(tmp_black,tmp_black,5);
  return tmp_black;
}

Point getPos(Mat &img)
{
  Point pos=Point(0,0);
  Mat tmp_canny;
  vector<vector<Point>> tmp_contours;
  vector<Point> target_contours;
  vector<Vec4i> tmp_hierarchy;
  double maxArea=0.0;
  double tmpArea=0.0;

  Canny(img,tmp_canny,80,160);
  //morphologyEx(tmp_canny,tmp_canny,MORPH_CLOSE,element);//对输出的边缘图像闭操作
  findContours(tmp_canny,tmp_contours,tmp_hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
  for(auto item:tmp_contours)
  {
    //cout<<contourArea(item)<<endl;
    tmpArea= contourArea(item);
    if(tmpArea>1)
    {
      if(tmpArea>=maxArea)
      {
        target_contours.assign(item.begin(),item.end());
        maxArea=tmpArea;
      }
    }
  }
  if(!target_contours.empty())
  {
    Moments M= moments(target_contours);
    double cX = double(M.m10 / M.m00);
    //求取轮廓重心的Y坐标
    double cY = double(M.m01 / M.m00);
    pos.x=cX; pos.y=cY;
    cv::Rect rect = cv::boundingRect(cv::Mat(target_contours));
    rectangle(frame,rect,Scalar(255,0,0));
    circle(frame,Point(rect.x+rect.width/2,rect.y+rect.height/2),2,Scalar(255,0,0),-1);
  }

  return pos;

}

Point getPos(Mat &img,Mat&src_img,bool drawOut)
{
  Point pos=Point(0,0);
  Mat tmp_canny;
  vector<vector<Point>> tmp_contours;
  vector<Point> target_contours;
  vector<Vec4i> tmp_hierarchy;
  double maxArea=0.0;
  double tmpArea=0.0;

  Canny(img,tmp_canny,80,160);
  //morphologyEx(tmp_canny,tmp_canny,MORPH_CLOSE,element);//对输出的边缘图像闭操作
  findContours(tmp_canny,tmp_contours,tmp_hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
  for(auto item:tmp_contours)
  {
    //cout<<contourArea(item)<<endl;
    tmpArea= contourArea(item);
    if(tmpArea>1)
    {
      if(tmpArea>=maxArea)
      {
        target_contours.assign(item.begin(),item.end());
        maxArea=tmpArea;
      }
    }
  }
  if(!target_contours.empty())
  {
    Moments M= moments(target_contours);
    double cX = double(M.m10 / M.m00);
    //求取轮廓重心的Y坐标
    double cY = double(M.m01 / M.m00);
    pos.x=cX; pos.y=cY;
    cv::Rect rect = cv::boundingRect(cv::Mat(target_contours));
    if(drawOut)
    {
      rectangle(src_img,rect,Scalar(255,0,0));
      circle(src_img,Point(rect.x+rect.width/2,rect.y+rect.height/2),2,Scalar(255,0,0),-1);
    }   
  }

  return pos;

}

void init()
{
  //dst points init
  dst_point[0]=Point2f (0,0);
  dst_point[1]=Point2f (0,500);
  dst_point[2]=Point2f (500,0);
  dst_point[3]=Point2f (500,500);
  //corn_points init
  corn_point[0]=Point (10,10);
  corn_point[1]=Point (10,490);
  corn_point[2]=Point (490,10);
  corn_point[3]=Point (490,490);
  //capture init
  capture.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  capture.set(CAP_PROP_FRAME_WIDTH,1280);
  capture.set(CAP_PROP_FRAME_HEIGHT,720);
  capture.set(CAP_PROP_FPS,60);
  capture.set(CAP_PROP_BUFFERSIZE,1);
  capture.set(CAP_PROP_EXPOSURE,140);
  //capture.set(CAP_PROP_AUTO_EXPOSURE,1);
  //element init
  element= getStructuringElement(MORPH_RECT,Size(15,15));
  fd=serialOpen("/dev/ttyS0",115200);
}
void calibrate()
{

}

void taskHandler()
{
  //get the points
  warpPerspective(frame,dst,H,Size(500,500));
  circle(dst,Point(250,250),3,Scalar(255,0,0),-1);
  r_binary=splitRed(dst);
  g_binary=splitGreen(dst);
  rp= getPos(r_binary,dst,true);
  gp= getPos(g_binary,dst,true);
  //task select
  if(task_num==1)
  {
    x_err3=rp.x-250;y_err3=rp.y-250;
    int avg_x=(x_err1+x_err2+x_err3)/3;
    int avg_y=(y_err1+y_err2+y_err3)/3;
    serialPrintf(fd,"X%dY%dF",avg_x,avg_y);
    x_err1=x_err2;y_err1=y_err2;
    x_err2=x_err3; y_err2=y_err3;
    printf("X%dY%dF\n",avg_x,avg_y);
  }

  else if(task_num==2)
  {
    int r_size=route.size();
    //cout<<"r_size:"<<r_size<<endl;
    if(r_size)
    {
      Point tmp=route.at(r_size-1);
      cout<<"target:"<<tmp<<endl;
      x_err3=rp.x-tmp.x;
      y_err3=rp.y-tmp.y;
      int avg_x=(x_err1+x_err2+x_err3)/3;
      int avg_y=(y_err1+y_err2+y_err3)/3; 
      x_err1=x_err2;y_err1=y_err2;
      x_err2=x_err3; y_err2=y_err3;
      //printf("X%dY%dF\n",avg_x,avg_y);
      serialPrintf(fd,"X%dY%dF",avg_x,avg_y);
      if((abs(avg_x)<=10)&&(abs(avg_y)<=10))
        route.pop_back();
    }
    else
    {
      serialPrintf(fd,"TF");
      printf("TF\n");
    }

  }
  
}

void taskConfig(int number)
{
  if(number==1)
  {

  }
  else if(number==2)
  {
    route.push_back(Point(50,50));
    for(int i=0;i<41;i++)
    {
      route.push_back(Point(50,50+i*10));
    }
    cout<<"1:"<<route.at(route.size()-1)<<endl;
    for(int i=0;i<41;i++)
    {
      route.push_back(Point(50+i*10,450));
    }
    cout<<"2:"<<route.at(route.size()-1)<<endl;
    for(int i=0;i<41;i++)
    {
      route.push_back(Point(450,450-i*10));
    }
    cout<<"3:"<<route.at(route.size()-1)<<endl;
    for(int i=0;i<41;i++)
    {
      route.push_back(Point(450-i*10,50));
    }
    cout<<"4:"<<route.at(route.size()-1)<<endl;

  }
}