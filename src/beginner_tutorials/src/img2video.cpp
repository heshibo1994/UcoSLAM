#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sstream>
using namespace std;
using namespace cv;
int main(int argc, char **argv)
{
    cv::VideoWriter writer;
    writer.open(string(argv[1])+"//video.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, cv::Size(2048,1536), true);
    for (int frameNumber = atoi(argv[2]); frameNumber<=atoi(argv[3]);frameNumber++) {
        string path = string(argv[1])+"//"+to_string(frameNumber)+".jpg";
        cout<<path<<endl;
        cv::Mat im = imread(path);
        writer.write(im);
    }
  return 0;
}