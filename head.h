#include <iostream>
#include "opencv2/opencv.hpp"
#include "wiringPi.h"
#include "wiringSerial.h"
#include <algorithm>
using namespace std;
using namespace cv;
void Calibrator(Mat &InputImage,Mat &OutputImage);