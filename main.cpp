#include <iostream>
#include "opencv2/opencv.hpp"
#include "wiringPi.h"
#include "wiringSerial.h"
#include <algorithm>
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
Scalar black_h=Scalar(110,255,255);

vector<Mat> channels;
Point getPos(Mat &img);
void init();
Mat splitRed(Mat &input);
Mat splitGreen(Mat &input);
Mat splitBlcak(Mat &input);
Point2f getPos(Mat &img,Mat&src_img,bool drawOut);
void getRect(Mat &img,Mat &src_img,Point2f *points,bool drawOut);
void getRect2(Mat &img,Mat &src_img,Point2f *points,bool drawOut);
void getRoute(Point2f *points,float step_len,vector<Point2f> &route);
void taskHandler();
void taskConfig(int number);
Point p1,p2,p3;
Point rp,gp;
Point2f cornPts[4];
Mat H;

int fd;

int cnt=0;
bool save=false;
bool calib_flag=false;
bool ready_flag=false;
Point2f src_point[4];
Point2f dst_point[4];
Point corn_point[4];
int corn_index[4];
int send_cnt=0;
float x_err1,x_err2,x_err3;
float y_err1,y_err2,y_err3;
Mat dst;
int opt=0;

int task_num=2;

vector<Point> route;
vector<Point2f> route2;
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
        if(ready_flag)
        {
          taskHandler();
          // getRect2(black,dst,cornPts,false);
          // getRoute(cornPts,10.0,route2);
          // for(auto item:route2)
          // {
          //   circle(dst,item,2,Scalar(255,0,0),-1);
          // }
          // route2.clear();
          imshow("black",black);
          imshow("dst",dst);
          int64 t2=cv::getTickCount();
          //cout<<"tim:"<<(t2-t1)/cv::getTickFrequency()<<endl;
        }
        
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
        p1= getPos(binary,frame,true);
        p2= getPos(binary2,frame,true);
        imshow("frame",frame);
        imshow("binary",binary);

        imshow("binary2",binary2);
      }

      opt=waitKey(1);
      if(opt==32)
      {
        src_point[cnt]=p1;
        cnt++;
        cout<<"cnt:"<<cnt<<"points:"<<p1<<endl;
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
      if(opt==66)
      {
        route2.clear();
        getRect2(black,dst,cornPts,true);
        getRoute(cornPts,3.0,route2);
        for(auto item:route2)
        {
          cout<<item<<endl;
        }
      }
    }
    if(serialDataAvail(fd))
    {
      char tmp_ch=serialGetchar(fd);
      printf("get %c\n",tmp_ch);
      if((tmp_ch>=1)&&(tmp_ch<=7))
      {
        ready_flag=false;
        task_num=tmp_ch;
        taskConfig(task_num);
        printf("task %d\n",task_num);
      }
      if(tmp_ch=='S')
      {
        ready_flag=true;
      }
      if(tmp_ch=='Z')
      {
        ready_flag=false;
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
  //Mat tmp_element=getStructuringElement(MORPH_RECT,Size(9,9));
  //erode(tmp_black,tmp_black,tmp_element);
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

Point2f getPos(Mat &img,Mat&src_img,bool drawOut)
{
  Point2f pos=Point2f(0,0);
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
    //float r;
    //cv::minEnclosingCircle(cv::Mat(target_contours),pos,r);
    if(drawOut)
    {
      circle(src_img,pos,2,Scalar(255,0,0),2,-1);
      // circle(src_img,pos,r,Scalar(255,0,0),1);
      // circle(src_img,pos,2,Scalar(255,0,0),-1);
      //rectangle(src_img,rect,Scalar(255,0,0));
      //circle(src_img,Point(rect.x+rect.width/2,rect.y+rect.height/2),2,Scalar(255,0,0),-1);
    }   
  }

  return pos;

}

void getRect(Mat &img,Mat &src_img,Point2f *points,bool drawOut)
{
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(img,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());
  if(!contours.empty())
  {
    Point2f P[3][4];
    int i=0;
    for (auto item1 : contours)
    {
      cout << "countours " << i << "area is " << contourArea(item1) << endl;
      if(contourArea(item1)>50000)
      {
        RotatedRect rect = cv::minAreaRect(cv::Mat(item1));
        rect.points(P[i]);
        i++;
      }
    }
    i = 0;

    for (int j = 0; j < 4; j++)
    {
      P[2][j].x = P[0][j].x / 2.0 + P[1][j].x / 2.0;
      P[2][j].y = P[0][j].y / 2.0 + P[1][j].y / 2.0;
    }

    for (int j = 0; j < 4; j++)
    {
      points[j]=P[2][j];

      if(drawOut) {
        line(src_img, P[0][j], P[0][(j + 1) % 4], Scalar(0, 0, 255), 1);  //绘制最小外接矩形每条边
        line(src_img, P[1][j], P[1][(j + 1) % 4], Scalar(0, 0, 255), 1);  //绘制最小外接矩形每条边
        line(src_img, P[2][j], P[2][(j + 1) % 4], Scalar(255, 0, 0), 1);  //绘制最小外接矩形每条边
      }
    }
  }
  else
  {
    cout<<"no enough contours"<<endl;
  }
}
void getRect2(Mat &img,Mat &src_img,Point2f *points,bool drawOut)
{
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  vector<Point> out;
  findContours(img,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE,Point());
  if(!contours.empty())
  {
    Point2f P[3][4];
    int i=0;
    for (auto item1 : contours)
    {
      //cout << "countours " << i << "area is " << contourArea(item1) << endl;
      if(contourArea(item1)>50000)
      {
        approxPolyDP(item1,out,50,true);
      }
      if(out.size()==4)
      {
        int j=0;
        for(auto tmp:out)
        {
          cout<<tmp<<endl;
          P[i][j]=tmp;
          j++;
          circle(src_img,tmp,2,Scalar(255,0,0),-1);
        }
        //cout<<"next:"<<i<<endl;
        i++;
      }
    }
    for(int k=0;k<4;k++)
    {
      int index=0;
      int min_dis=10000;
      for(int l=0;l<4;l++)
      {
        int tmp=pow(P[0][k].x-P[1][l].x,2)+pow(P[0][k].y-P[1][l].y,2);
        if(tmp<min_dis)
        {
          min_dis=tmp;
          index=l;
        }
      }
      P[2][k].x=P[0][k].x/2.0+P[1][index].x/2.0;
      P[2][k].y=P[0][k].y/2.0+P[1][index].y/2.0;
      points[k]=P[2][k];
      //circle(src_img,P[2][k],2,Scalar(0,0,255),2,-1);
    }
    
  }
  else
  {
    cout<<"no enough contours"<<endl;
  }
}

void getRoute(Point2f *points,float step_len,vector<Point2f> &tmp_route)
{
  Point2f start_p,end_p;
  float k,theat;
  float cos_theta,sin_theta;

  int step_num=0;

  for(int i=0;i<4;i++)
  {
    start_p=points[i];end_p=points[(i+1)%4];

    float x_dis=fabs(start_p.x-end_p.x);
    float y_dis=fabs(start_p.y-end_p.y);

    k=(start_p.x-end_p.x)/(start_p.y-end_p.y);
    //cout<<"k is: "<<k<<endl;
    theat= atan(k);
    //cout<<"theat is "<<theat<<endl;
    float distance=powf((start_p.x-end_p.x),2)+powf((start_p.y-end_p.y),2);
    distance= sqrtf(distance);


    step_num=int(distance/step_len);
    float x_off=step_len* cos(theat);
    float y_off=step_len* sin(theat);
    


      for(int j=0;j<step_num;j++)
      {
        if((start_p.x>=end_p.x)&&(start_p.y>=end_p.y))
        {
          tmp_route.push_back(Point2f(start_p.x-j*step_len*x_dis/distance,start_p.y-j*step_len*y_dis/distance));
        }
        else if((start_p.x>=end_p.x)&&(start_p.y<end_p.y))
        {
          tmp_route.push_back(Point2f(start_p.x-j*step_len*x_dis/distance,start_p.y+j*step_len*y_dis/distance));

        }
        else if((start_p.x<end_p.x)&&(start_p.y>=end_p.y))
        {
          tmp_route.push_back(Point2f(start_p.x+j*step_len*x_dis/distance,start_p.y-j*step_len*y_dis/distance));

        }
        else 
        {
          tmp_route.push_back(Point2f(start_p.x+j*step_len*x_dis/distance,start_p.y+j*step_len*y_dis/distance));
        }
      }
      for(int i=0;i<8;i++)
      {
        tmp_route.push_back(end_p);
      }     
    
  }
  reverse(tmp_route.begin(),tmp_route.end());

}
void init()
{
  src_point[0]=Point2f(479,123);
  src_point[1]=Point2f(495,658);
  src_point[2]=Point2f(1043,116);
  src_point[3]=Point2f(1033,657);
  //dst points init
  dst_point[0]=Point2f (0,0);
  dst_point[1]=Point2f (0,600);
  dst_point[2]=Point2f (600,0);
  dst_point[3]=Point2f (600,600);
  // H= getPerspectiveTransform(src_point,dst_point);
  // calib_flag=true;
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
  element= getStructuringElement(MORPH_RECT,Size(31,31));
  fd=serialOpen("/dev/ttyS0",115200);
}

void taskHandler()
{
  //get the points
  warpPerspective(frame,dst,H,Size(600,600));
  circle(dst,Point(250,250),3,Scalar(255,0,0),-1);
  r_binary=splitRed(dst);
  imshow("r_binary",r_binary);
  g_binary=splitGreen(dst);
  black=splitBlcak(dst);
  rp= getPos(r_binary,dst,true);
  gp= getPos(g_binary,dst,true);
  //task select
  if(task_num==1)
  {

      x_err3=rp.x-300;y_err3=rp.y-300;
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
      //cout<<"target:"<<tmp<<endl;
        circle(dst,tmp,2,Scalar(0,0,255),2,-1);
        x_err3=rp.x-tmp.x;
        y_err3=rp.y-tmp.y;
        float avg_x=(x_err1+x_err2+x_err3)/3.0;
        float avg_y=(y_err1+y_err2+y_err3)/3.0; 
        x_err1=x_err2;y_err1=y_err2;
        x_err2=x_err3; y_err2=y_err3;
        //printf("X%dY%dF\n",avg_x,avg_y);
        serialPrintf(fd,"X%.2fY%.2fF",avg_x,avg_y);
        //if((fabs(avg_x)<=10)&&(fabs(avg_y)<=10))
          route.pop_back();
      
    }
    else
    {
      serialPrintf(fd,"TF");
      //printf("TF\n");
    }

  }
  else if(task_num==3||task_num==4)
  {
    int r_size=route2.size();
    //cout<<"r_size:"<<r_size<<endl;
    if(r_size)
    {
      //if(!(rp.x==0||rp.y==0))
      //{
        send_cnt++;
        if(send_cnt==2)
        {
          send_cnt=0;
        Point2f tmp=route2.at(r_size-1);
        //cout<<"target:"<<tmp<<endl;
        circle(dst,tmp,2,Scalar(0,0,255),2,-1);
        x_err3=rp.x-tmp.x;
        y_err3=rp.y-tmp.y;
        float avg_x=(x_err1+x_err2+x_err3)/3.0;
        float avg_y=(y_err1+y_err2+y_err3)/3.0; 
        x_err1=x_err2;y_err1=y_err2;
        x_err2=x_err3; y_err2=y_err3;
        printf("X%.2fY%.2fF\n",avg_x,avg_y);
        serialPrintf(fd,"X%.2fY%.2fF",avg_x,avg_y);
        //if((fabs(avg_x)<=10)&&(fabs(avg_y)<=10))
        route2.pop_back();
    }
      //}
    }
    else
    {
      serialPrintf(fd,"TF");
      //printf("TF\n");
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
    route.clear();
    route.push_back(Point(60,60));
    for(int i=0;i<97;i++)
    {
      route.push_back(Point(60,60+i*5));
    }
    cout<<"1:"<<route.at(route.size()-1)<<endl;
    for(int i=0;i<97;i++)
    {
      route.push_back(Point(60+i*5,540));
    }
    cout<<"2:"<<route.at(route.size()-1)<<endl;
    for(int i=0;i<97;i++)
    {
      route.push_back(Point(540,540-i*5));
    }
    cout<<"3:"<<route.at(route.size()-1)<<endl;
    for(int i=0;i<97;i++)
    {
      route.push_back(Point(540-i*5,60));
    }
    cout<<"4:"<<route.at(route.size()-1)<<endl;
  }
  else if(number==3||number==4)
  {
    route2.clear();
    getRect2(black,dst,cornPts,true);
    getRoute(cornPts,3.0,route2);
    for(auto item:route2)
    {
      cout<<item<<endl;
    }
  }
}