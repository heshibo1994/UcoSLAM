#include "flycapture/FlyCapture2.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class PointgreyStream{

public:
// define the public variables

  cv::Mat curr_image;

// define the parameters for camera calibration

  struct calibration {
      double fx;  //  focal length (in pixels)
      double fy;  //  focal length (in pixels)
      double cx;  //  principal point (u-coordinate)
      double cy;  //  principal point (v-coordinate)
      calibration () {
        fx = 0.0;
        fy = 0.0;
        cx = 0.0;
        cy = 0.0;
      }
  };

//
//
// functions
//
//
//

  //constructor
  PointgreyStream(){

    error = camera.Connect( 0 );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to connect to pointgrey camera : no device" << std::endl;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from pointgrey camera" << std::endl;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " "
              << camInfo.serialNumber <<" " 
              << camInfo.isColorCamera<< std::endl;

    error = camera.StartCapture();
    if ( error == FlyCapture2::PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
    }
    else if ( error != FlyCapture2::PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;
    }
    error = camera.SetVideoModeAndFrameRate( FlyCapture2::VIDEOMODE_1024x768Y16 , FlyCapture2::FRAMERATE_30);
    if(error!= FlyCapture2::PGRERROR_OK){
      std::cout<< "Failed to set the frame rate and image size"<<std::endl;
    }
   
  };
  //destructor
  ~PointgreyStream(){

    error = camera.StopCapture();
    if ( error != FlyCapture2::PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show
        // an error message
    }

    camera.Disconnect();
  };
  //7 0.5 20
  void pointgreyInit(const int _SHUTTER=7, const int _BRIGHTNESS=0.5, const float _GAIN=20.0){
    prop.type = FlyCapture2::SHUTTER;
    //Ensure the property is on.
    prop.onOff = true;
    //Ensure auto-adjust mode is off.
    prop.autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    prop.absControl = true;
    //Set the absolute value of shutter to 20 ms.
    prop.absValue = _SHUTTER;
    error = camera.SetProperty( &prop );

    //Set the property.
    prop.type = FlyCapture2::BRIGHTNESS;
    prop.absControl = true;
    prop.absValue =_BRIGHTNESS;


    error = camera.SetProperty( &prop );

    prop.type = FlyCapture2::GAIN;
    prop.autoManualMode = false;
    prop.absControl = true;
    prop.absValue=_GAIN;
    error = camera.SetProperty( &prop );
  }

  void getData(){
    error = camera.RetrieveBuffer(&rawImage);
    if ( error != FlyCapture2::PGRERROR_OK )
    {
            std::cout << "pointgrey capture error" << std::endl;
    }
    else{
      rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

      unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
      curr_image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

    }
  };

private:
  FlyCapture2::Error error;
  FlyCapture2::Camera camera;
  FlyCapture2::CameraInfo camInfo;
  FlyCapture2::Property prop;
  FlyCapture2::Image rawImage;
  FlyCapture2::Image rgbImage;



};
