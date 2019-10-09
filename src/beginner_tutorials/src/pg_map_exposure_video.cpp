#include "ros/ros.h"
#include "unistd.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include "flycapture/FlyCapture2.h"
#include "pointgreyStream.h"
#include <sstream>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <math.h>
#include <Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <mynteyed/camera.h>
//#include <mynteyed/utils.h>

#include <ucoslam/ucoslam.h>
#include <ucoslam/mapviewer.h>
using namespace std;
using namespace Eigen;
//MYNTEYE_USE_NAMESPACE

std::map<int,Eigen::Matrix<double,4,4>> markermap;//二维码在真实世界中的位姿 
vector<int> markers_id ;
double markersize ;
int SHUTTER=7 ; int BRIGHTNESS=0.5; float GAIN=20.0;
bool FLAG = false;
ucoslam::UcoSlam SLAM;//The main class
ucoslam::Params UcoSlamParams;//processing parameters
ucoslam::ImageParams cameraParams;//camera parameters
ucoslam::MapViewer MapViwer;//Viewer to see the 3D map and the input images
std::shared_ptr<ucoslam::Map> globalmap=std::make_shared<ucoslam::Map>();
//PointgreyStream *mypointgrey = new PointgreyStream();
char keyPressed=0;
Eigen::Matrix<double,4,4> cam_pose_matrix ;
Eigen::Matrix<double,4,4> marker_pose_matrix;
Eigen::Matrix<double,4,4> t;//最终结果,相机在真实世界下的位姿
int mindistance_id = 0;

//载入真实世界地图
void loadMarkermap(string path){
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if(!fs.isOpened()) throw std::runtime_error(std::string(__FILE__)+" could not open file:"+path);
    fs["markersize"] >> markersize;
    fs["markers_id"] >> markers_id;
    for (std::size_t  i =0;i<markers_id.size();i++){
        cv::Mat cvmarker ;
        Eigen::Matrix<double,4,4> marker;
        fs["id_"+to_string(markers_id[i])] >> cvmarker;
        cv2eigen(cvmarker,marker);
        markermap[markers_id[i]] = marker;
    }
    cout<<"==========真实世界坐标系下二维码位姿态=========="<<endl;
    std::map<int, Eigen::Matrix<double,4,4>>::const_iterator it; 
    for (it = markermap.begin(); it != markermap.end(); ++it){
        cout << "二维码编号："<< it->first <<endl;
        cout << "二维码位姿："<< endl;
        cout << it->second << endl;
        cout <<"==============================="<<endl;
    }
    cout<<endl;
}

//slam启动和参数设置
void setSlam(int argc,string cameraparmas_path,string direction_path,string map_path){
    UcoSlamParams.runSequential=false;//run in sequential mode to avoid skipping frames
    UcoSlamParams.detectMarkers=true;//no markers in this example.
    UcoSlamParams.aruco_markerSize=markersize;
    UcoSlamParams.aruco_Dictionary="ARUCO_MIP_36h12";
    UcoSlamParams.nthreads_feature_detector =1;
    //载入配置文件,相机内参---------------------------------------------2
    cameraParams.readFromXMLFile(cameraparmas_path);
    //开始准备运行UcoSlam，并载入字典文件------------------------------------------3
    //创建地图文件，并载入历史地图或者空地图-----------------------------------------4
    if(argc==7){
        fstream file;
        file.open(map_path,ios::in);
        if(file)
            globalmap->readFromFile(map_path);
    }
    SLAM.setParams(globalmap,UcoSlamParams,direction_path);
}

//相机驱动初始化与设置
// void setCamera(){
//     for (int frameNumber = 0; ;frameNumber++) {
//         char key = static_cast<char>(cv::waitKey(1));
//         if ( key =='w' || key == 'W') {break;}
//         if ( key =='u' || key == 'U'){SHUTTER =SHUTTER+1;}
//         if ( key =='j' || key == 'J'){SHUTTER =SHUTTER-1;}
//         if ( key =='i' || key == 'I'){BRIGHTNESS =BRIGHTNESS+1;}
//         if ( key =='k' || key == 'K'){BRIGHTNESS =BRIGHTNESS-1;}
//         if ( key =='o' || key == 'O'){GAIN =GAIN+1;}
//         if ( key =='l' || key == 'L'){GAIN =GAIN-1;}
//         mypointgrey->pointgreyInit(SHUTTER,BRIGHTNESS,GAIN);
//         mypointgrey->getData();
//         cv::Mat im =  mypointgrey->curr_image;
//         cv::putText(im, "SHUTTER: "+to_string(SHUTTER), cv::Point(1000,500), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
//         cv::putText(im, "BRIGHTNESS: "+to_string(BRIGHTNESS), cv::Point(1000,600), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
//         cv::putText(im, "GAIN: "+to_string(GAIN), cv::Point(1000,700), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);
//         cv::imshow("im",im); 
//     }
// }

//运行slam
void startSlam( cv::Mat im,string record_path,int frameNumber){
    //保存图像--------------------------------------------------------------6
    cv::Mat posef2g= SLAM.process(im,cameraParams,frameNumber);
    if(posef2g.empty()){
        std::cerr<<"Frame "<<frameNumber<<" pose not found"<<std::endl;
        t <<0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    else{
        std::cerr<<"Frame "<<frameNumber<<endl;
        //cout<<" pose "<<posef2g<<std::endl;
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

        if (globalmap->map_markers.count(mindistance_id) != 0){
            cv2eigen(globalmap->map_markers[mindistance_id].pose_g2m,marker_pose_matrix);
            t = markermap[mindistance_id]*marker_pose_matrix.inverse()*cam_pose_matrix.inverse(); 
            cout<<t<<endl;  
        }  
    }
    keyPressed=MapViwer.show(globalmap,im,posef2g);  
    cv::putText(im, to_string(frameNumber)+" x "+to_string(t(0,3))+" y "+to_string(t(1,3))+" z "+to_string(t(2,3)), cv::Point(500,500), cv::FONT_HERSHEY_COMPLEX, 2, cv::Scalar(0, 255, 255), 2, 8, 0);  
    cv::imwrite(record_path+"//"+to_string(frameNumber)+".jpg",im);
}

//发布
void publishPose(ros::Publisher odom_pub){
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
    odom_pub.publish(odometry);  
    ros::spinOnce();        
}

int main(int argc, char **argv)  
{  

    loadMarkermap(string(argv[1]));

    setSlam(argc,string(argv[2]),string(argv[3]),string(argv[4]));

    //setCamera();

    //ros系统的初始化，topic话题的设定×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××
    ros::init(argc, argv, "talker");  
    ros::NodeHandle n;  
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);  //发布4×4矩阵    
    ros::Rate loop_rate(10);  
    
    //mypointgrey->pointgreyInit(SHUTTER,BRIGHTNESS,GAIN);
    cv::VideoCapture VideoIn;//video capturer
    VideoIn.open(argv[6]);
    if(!VideoIn.isOpened()) throw std::runtime_error("Could not open video:"+string(argv[6]));
    cv::Mat inputImage;
    while( VideoIn.grab() && keyPressed!=27){//keyPressed ==27 is esc
            VideoIn.retrieve(inputImage);
            int frameNumber=VideoIn.get(CV_CAP_PROP_POS_FRAMES);
            startSlam(inputImage,string(argv[5]),frameNumber);
            publishPose(odom_pub); 
    }

    // for (int frameNumber = 0; keyPressed!=27;frameNumber++) {
    //     char key = static_cast<char>(cv::waitKey(1));
    //     if (key == 27 || key == 'q' || key == 'Q') {break;} 
    //     mypointgrey->getData();
    //     cv::Mat im =  mypointgrey->curr_image;
    //     startSlam(im,string(argv[5]),frameNumber);
    //     publishPose(odom_pub);   
    // }

    if(argc==6){globalmap->saveToFile(string(argv[4]));}
    if(argc==7){globalmap->saveToFile(string(argv[6]));}

    cv::destroyAllWindows();  
    return 0;
}  






