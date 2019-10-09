#include <iostream>
#include "ros/ros.h"
#include "flycapture/FlyCapture2.h"
#include "pointgreyStream.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sstream>
using namespace std;
using namespace cv;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "record");
  PointgreyStream *mypointgrey = new PointgreyStream();
  int SHUTTER = 7 ; 
  int BRIGHTNESS = 0.5; 
  float GAIN = 20.0;
  bool FLAG = false;
  mypointgrey->pointgreyInit(SHUTTER,BRIGHTNESS,GAIN);
  char keyPressed=0;
  cv::VideoWriter writer;
  Size size = Size(2048,1536);
  writer.open(string(argv[1])+"video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, size, true);
  for (int frameNumber = 0; keyPressed!=27;frameNumber++) {
      char key = static_cast<char>(cv::waitKey(1));
      if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
          break;
      }
      if ( key =='u' || key == 'U'){
          SHUTTER =SHUTTER+1;
      }
      if ( key =='j' || key == 'J'){
          SHUTTER =SHUTTER-1;
      }

      if ( key =='i' || key == 'I'){
          BRIGHTNESS =BRIGHTNESS+1;
      }
      if ( key =='k' || key == 'K'){
          BRIGHTNESS =BRIGHTNESS-1;
      }
      if ( key =='o' || key == 'O'){
          GAIN =GAIN+1;
      }
      if ( key =='l' || key == 'L'){
          GAIN =GAIN-1;
      }
      if ( key =='s' || key == 'S'){
          FLAG = !FLAG;
      }
      mypointgrey->pointgreyInit(SHUTTER,BRIGHTNESS,GAIN);
      mypointgrey->getData();
      
      cv::Mat im =  mypointgrey->curr_image;
      
      if(FLAG){
          cv::imwrite(string(argv[1])+to_string(frameNumber)+".jpg",im);
          writer.write(im);
      }

      cv::Point origin; 
      origin.x = 1000;
      origin.y = 500;
      string a = FLAG ? "RECORDING":"NO RECORD";
      cv::putText(im, "SHUTTER: "+to_string(SHUTTER), Point(1000,500), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
      cv::putText(im, "BRIGHTNESS: "+to_string(BRIGHTNESS), Point(1000,600), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
      cv::putText(im, "GAIN: "+to_string(GAIN), Point(1000,700), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
      cv::putText(im, a, Point(1000,800), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);

      cv::imshow("im",im); 
      
  }
  return 0;
}
