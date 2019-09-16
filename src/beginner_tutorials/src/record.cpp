#include <iostream>
#include "ros/ros.h"
#include "flycapture/FlyCapture2.h"
#include "pointgreyStream.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
using namespace std;
using namespace cv;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "record");
  PointgreyStream *mypointgrey = new PointgreyStream();
  mypointgrey->pointgreyInit(7,0.5,20);
  char keyPressed=0;
  cv::VideoWriter writer;
  Size size = Size(2048,1536);
  writer.open(string(argv[1])+"video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, size, true);
  for (int frameNumber = 0; keyPressed!=27;frameNumber++) {
    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
        break;
    }
    mypointgrey->getData();
    cv::Mat im =  mypointgrey->curr_image;
    cv::imshow("im",im);
    cv::imwrite(string(argv[1])+to_string(frameNumber)+".jpg",im);
    writer.write(im);
  }
  return 0;
}