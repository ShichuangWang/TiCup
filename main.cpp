#include <iostream>
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

VideoCapture capture(0);
Mat frame,lab,red,green,r_ch,g_ch,binary,binary2,tmp,tmp2,black;

Scalar red_l=Scalar(80,140,80);
Scalar red_h=Scalar(190,250,160);

Scalar green_l=Scalar(80,80,80);
Scalar green_h=Scalar(160,160,160);

Scalar black_l=Scalar(0,0,0);
Scalar black_h=Scalar(15,255,255);

vector<Mat> channels;
Point getPos(Mat &img);
void init();
Point p1,p2;
Mat H;

int cnt=0;
bool calib_flag=false;
Point2f src_point[4];
Point2f dst_point[4];

Mat dst;
int opt=0;
int main() {

  init();
  capture.set(CAP_PROP_FOURCC,VideoWriter::fourcc('M','J','P','G'));
  capture.set(CAP_PROP_FRAME_WIDTH,1280);
  capture.set(CAP_PROP_FRAME_HEIGHT,720);
  capture.set(CAP_PROP_FPS,60);
  capture.set(CAP_PROP_BUFFERSIZE,1);
  capture.set(CAP_PROP_EXPOSURE,-6);
  //capture.set(CAP_PROP_AUTO_EXPOSURE,1);
  Mat element= getStructuringElement(MORPH_RECT,Size(15,15));

  cout<<"CAP_PROP_EXPOSURE:"<<capture.get(CAP_PROP_EXPOSURE)<<endl;
  while (true)
  {
    capture>>frame;
    if(!frame.empty())
    {
      if(calib_flag)
      {
        warpPerspective(frame,dst,H,Size(500,500));
        imshow("dst",dst);
      }
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
      p1= getPos(binary);
      p2= getPos(binary2);
      imshow("frame",frame);
      //imshow("red",red);
      //imshow("r_ch",r_ch);
      //imshow("g_ch",g_ch);
      //bitwise_or(binary,binary2,binary);
      //imshow("tmp",tmp);
      imshow("binary",binary);

      //imshow("binary2",binary2);
      //imshow("black",black);
      //imshow("green",green);
      opt=waitKey(1);
      if(opt==32)
      {
        src_point[cnt]=p2;
        cnt++;
        cout<<"cnt:"<<cnt<<endl;
        if(cnt==4)
        {
          cout<<"get all point"<<endl;
          H= getPerspectiveTransform(src_point,dst_point);
          calib_flag= true;
        }
      }
    }
  }

  return 0;
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

void init()
{
  dst_point[0]=Point2f (0,0);
  dst_point[1]=Point2f (0,500);
  dst_point[2]=Point2f (500,0);
  dst_point[3]=Point2f (500,500);
}
void calibrate()
{

}