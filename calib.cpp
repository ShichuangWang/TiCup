#include "head.h"

void Calibrator(Mat &InputImage,Mat &OutputImage){

  //2.8mm非原装镜头畸变参数,分辨率1280X720

  double d_cameraMatrix[3][3]={{840.3773,0,702.079},{0,840.2290324752250,340.3635427612422},{0,0,1}};
  double d_disCoeffs[1][5]={-0.460099954467291,0.207352742152567,0,0,0};

  Size image_size=InputImage.size();
  Mat map_x;
  Mat map_y;
  map_x.create(image_size,CV_32FC1);
  map_y.create(image_size,CV_32FC1);
  Mat R=Mat::eye(3,3,CV_32F);
  Mat cameraMatrix(3,3,CV_64FC1,d_cameraMatrix);  //内参矩阵
  Mat disCoeffs(1,5,CV_64FC1,d_disCoeffs);        //畸变系数

  Mat newCameraMatrix= getOptimalNewCameraMatrix(cameraMatrix,disCoeffs,image_size,0,image_size);
  initUndistortRectifyMap(cameraMatrix,disCoeffs,R,newCameraMatrix,InputImage.size(),CV_32FC1,map_x,map_y);
  remap(InputImage,OutputImage,map_x,map_y,INTER_LINEAR);

  //undistort(InputImage,OutputImage,cameraMatrix,disCoeffs);
}