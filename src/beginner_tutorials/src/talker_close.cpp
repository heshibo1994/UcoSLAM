
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>

#include <sstream>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <math.h>
#include <Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <mynteyed/camera.h>
#include <mynteyed/utils.h>

#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
using namespace std;
using namespace Eigen;
MYNTEYE_USE_NAMESPACE


int main(int argc, char **argv)  
{  
    //设定slam系统的输入文件，视频流文件，配置文件，字典文件，地图文件×××××××××××××××××××××××××××××××××××××××××××××
    //if(argc!=5) throw std::runtime_error("usage : inputmap parameters.yml vocabulary.fbow  outputmap.map");
    //cv::VideoCapture VideoIn;//video capturer
    ucoslam::UcoSlam SLAM;//The main class
    ucoslam::Params UcoSlamParams;//processing parameters
    ucoslam::ImageParams cameraParams;//camera parameters
    ucoslam::MapViewer MapViwer;//Viewer to see the 3D map and the input images
    //创建地图文件，并载入地图
    std::shared_ptr<ucoslam::Map> globalmap=std::make_shared<ucoslam::Map>();
    //map->loadFromFile(pathToFile);
    // fstream file;
    // file.open(argv[4],ios::in);
    // if(file)
  	//     globalmap->readFromFile(argv[4]);
    //载入配置文件
    cameraParams.readFromXMLFile(argv[2]);
    //ros系统的初始化，topic话题的设定×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
    ros::init(argc, argv, "talker");  
    ros::NodeHandle n;  
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);  //发布4×4矩阵
    ros::Rate loop_rate(10);  

    //相机驱动初始化与设置××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
    //打开相机
    Camera cam;
    DeviceInfo dev_info;
    if (!util::select(cam, &dev_info)) {
      return 1;
    }
    util::print_stream_infos(cam, dev_info.index);
    std::cout << "Open device: " << dev_info.index << ", "<< dev_info.name << std::endl << std::endl;
    OpenParams params(dev_info.index);
    //参数设置
    params.depth_mode = DepthMode::DEPTH_COLORFUL;
    params.stream_mode = StreamMode::STREAM_2560x720;
    params.ir_intensity = 0;
    params.framerate = 15;
    cam.Open(params);
    std::cout << std::endl;
    if (!cam.IsOpened()) {
      std::cerr << "Error: Open camera failed" << std::endl;
      return 1;
    }
    std::cout << "Open device success" << std::endl << std::endl;
    cv::namedWindow("left");
    //载入真实世界地图
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::runtime_error(std::string(__FILE__)+" could not open file:"+argv[1]);
    std::map<int,Eigen::Matrix<double,4,4>> markermap;//二维码在真实世界中的位姿
    vector<int> markers_id ;
    double markersize ;
    fs["markersize"] >> markersize;
    fs["markers_id"] >> markers_id;
    for (int i =0;i<markers_id.size();i++){
        cv::Mat cvmarker ;
        Eigen::Matrix<double,4,4> marker;
        fs["id_"+to_string(markers_id[i])] >> cvmarker;
        cv2eigen(cvmarker,marker);
        markermap[markers_id[i]] = marker;
    }
    cout<<"aaaaaaaaaaaaaaaaaa"<<endl;
    cout<<"==========真实世界坐标系下二维码位姿态=========="<<endl;
    std::map<int, Eigen::Matrix<double,4,4>>::const_iterator it; 
    for (it = markermap.begin(); it != markermap.end(); ++it){
        cout << "二维码编号："<< it->first <<endl;
        cout << "二维码位姿："<< endl;
        cout << it->second << endl;
        cout <<"==============================="<<endl;
    }
    cout<<endl;


    cout<<"11111111111111"<<endl;
    //slam参数设置
    UcoSlamParams.runSequential=false;//run in sequential mode to avoid skipping frames
    UcoSlamParams.detectMarkers=true;//no markers in this example.
    UcoSlamParams.aruco_markerSize=markersize;
    UcoSlamParams.aruco_Dictionary="ARUCO_MIP_36h12";
    UcoSlamParams.nthreads_feature_detector =1;
    //开始准备运行UcoSlam，并载入字典文件
    SLAM.setParams(globalmap,UcoSlamParams,argv[3]);
    //以相机输出作为slam系统输入××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
    char keyPressed=0;

    Eigen::Matrix<double,4,4> cam_pose_matrix;
    Eigen::Matrix<double,4,4> marker_pose_matrix;
    Eigen::Matrix<double,4,4> t;//最终结果,相机在真实世界下的位姿
    cam_pose_matrix <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    marker_pose_matrix <<  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    t  <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    int mindistance_id = 21;

    for (int frameNumber = 0;frameNumber<=20000 && keyPressed!=27;frameNumber++) {
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
            break;
        }

        auto left_color = cam.GetStreamData(ImageType::IMAGE_LEFT_COLOR); 
        if (left_color.img) {
          // cv::Mat left = left_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
          // cv::imshow("left", left);

            auto left_img = left_color.img->To(ImageFormat::COLOR_BGR);
            cv::Mat left(left_img->height(), left_img->width(), CV_8UC3,
            left_img->data());
            cv::imshow("left", left);
            cv::Mat posef2g= SLAM.process(left,cameraParams,frameNumber);

            if(posef2g.empty()){
                std::cerr<<"Frame "<<frameNumber<<" pose not found"<<std::endl;
                t <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            }


            else{
                std::cerr<<"Frame "<<frameNumber<<endl;//" pose "<<posef2g<<std::endl;
                cv2eigen(posef2g,cam_pose_matrix);
                //坐标系转换××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
                
                double distance =0.0;
                std::map<int, Eigen::Matrix<double,4,4>>::const_iterator it;
                double mindistance =  100;
                for (it = markermap.begin(); it != markermap.end(); ++it){
                    distance = sqrt((t(0,3)-it->second(0,3))*(t(0,3)-it->second(0,3))+
                                    (t(1,3)-it->second(1,3))*(t(1,3)-it->second(1,3))+
                                    (t(2,3)-it->second(2,3))*(t(2,3)-it->second(2,3)));
                    if (distance<mindistance){
                        mindistance  = distance;
                        mindistance_id = it->first;
                    }
                }
                cout<<"当前帧距离最近的二维码: "<<mindistance_id<<endl;
                cout<<"相机在世界坐标系下的位姿"<<endl;
                cv2eigen(globalmap->map_markers[mindistance_id].pose_g2m,marker_pose_matrix);
                t = markermap[mindistance_id]*marker_pose_matrix.inverse()*cam_pose_matrix.inverse();           
                cout<<t<<endl;                
            }

        
            std::cout << "发出位姿数据" << std::endl;
            Eigen::Quaterniond q = Eigen::Quaterniond(t.block<3,3>(0,0));      
            nav_msgs::Odometry odometry;


            ros::Time current_time= ros::Time::now();
            odometry.header.stamp = current_time;
            //odometry.header = header;
            odometry.header.frame_id = "world";
            odometry.pose.pose.position.x = t(0,3);
            odometry.pose.pose.position.y = t(1,3);
            odometry.pose.pose.position.z = t(2,3);
            odometry.pose.pose.orientation.x = q.x();
            odometry.pose.pose.orientation.y = q.y();
            odometry.pose.pose.orientation.z = q.z();
            odometry.pose.pose.orientation.w = q.w();
            //draw a mininimal interface in an opencv window
            //keyPressed=MapViwer.show(map,inputImage,posef2g);
            odom_pub.publish(odometry);  
            ros::spinOnce();  
            keyPressed=MapViwer.show(globalmap,left,posef2g);    
        }
    }
    //now,  save the map
    //globalmap->saveToFile(argv[4]);
    //std::cout << "globalmap save to " <<argv[4]<< std::endl;
    cam.Close();
    cv::destroyAllWindows();  
    return 0;
}  






