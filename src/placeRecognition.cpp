#include <math.h>
#include <vector>
#include <aloam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "lidarFactor.hpp"
#include "aloam_velodyne/common.h"
#include "aloam_velodyne/tic_toc.h"
#include "aloam_velodyne/dwdx.h"
#include "aloam_velodyne/xy2000_lb2000.h"

std::mutex mBuf;
struct pos{
    double x;
    double y;
    double yaw;
};

struct placeInformation{
    int id;
    double x;
    double y;
    double yaw;
    std::string img_file_name;
};

std::queue<pos> gtPos;
std::queue<cv::Mat> imgs;
std::vector<placeInformation> placeInfo;

cv::Ptr<cv::FeatureDetector> orb;

float imgResizeFac;
FILE * fp;
int imageID;
std::string data_path;
std::string file_name;
int disThr, angThr;

void getGPS(const aloam_velodyne::dwdx msg)
{

    double lon = msg.longitude * 0.000001;
    double lat = msg.latitude * 0.000001;
    int gx, gy;
    XY2000::LB2000_XY2000(lon*M_PI/180.0, lat*M_PI/180.0, gx, gy);

    pos curGtPos;
    curGtPos.x = gy*0.1;//北x东y地z, 正北为0度，正东90度
    curGtPos.y = gx*0.1;
    curGtPos.yaw = msg.heading * 0.01;//范围[0,360)

//    mBuf.lock();
    gtPos.push(curGtPos);
    if(gtPos.size()==10) gtPos.pop();
//    mBuf.unlock();

    std::time_t timeTmp;
    std::time(&timeTmp);
    printf("get gps %s\n", std::to_string(timeTmp).c_str());

}


void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    std::time_t timeTmp;
    std::time(&timeTmp);
    try{

        cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat
        int rows =  image.rows;
        int cols = image.cols;
//        mBuf.lock();
        imgs.push(image);
        if(imgs.size()==10) imgs.pop();
//        mBuf.unlock();
        printf("get image %d %d %s\n", rows,cols,std::to_string(timeTmp).c_str());
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert to image!");
    }
}

void process(){
    while(true){
        if(!imgs.empty() && !gtPos.empty()){
            cv::Mat img = imgs.back();
            cv::resize(img,img,cv::Size(img.cols*imgResizeFac,img.rows*imgResizeFac));

            cv::cvtColor(img,img,cv::COLOR_BGR2GRAY);
            double x = gtPos.back().x;
            double y = gtPos.back().y;
            double yaw = gtPos.back().yaw;

            std::vector<cv::KeyPoint> keyPoints;
            orb->detect(img,keyPoints);
            cv::drawKeypoints(img,keyPoints,img);

            cv::imshow("RGB",img);
            cv::waitKey(1);
            for(int i = 0; i < placeInfo.size();i++){
                double dis = sqrt((placeInfo[i].x - x)*(placeInfo[i].x - x) + (placeInfo[i].y - y)*(placeInfo[i].y - y));
                int angle = abs(int(placeInfo[i].yaw - yaw));
                angle = angle > 180 ? 360-angle:angle;
                if(dis < disThr && angle < angThr){
                    char words[100];
                    sprintf(words,"Recognized to origin place %d", i);
                    cv::Mat tmpImg = img.clone();
                    cv::putText(tmpImg,words, cv::Point(10,50),0,0.8,cv::Scalar(0,0,255),2);


                    sprintf(words,"Origin place %d", i);
                    cv::Mat originImg = cv::imread(placeInfo[i].img_file_name);
                    cv::putText(originImg,words, cv::Point(10,50),0,0.8,cv::Scalar(0,0,255),2);


                    cv::imshow("origin image",originImg);
                    cv::imshow("recognition result", tmpImg);

                    cv::waitKey(1);
                    break;
                }
                else{
                    if(i == placeInfo.size()-1){
                        cv::destroyWindow("recognition result");
                        cv::destroyWindow("origin image");
                    }

                }
            }

        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "placeRecognition");
    ros::NodeHandle nh;
    nh.param<std::string>("data_path", data_path, "/home/hsc/Desktop/placeRecognition");
    nh.param<float>("imgResizeFac", imgResizeFac, 0.5);
    nh.param<int>("disThr", disThr, 10);
    nh.param<int>("angThr", angThr, 50);
    imageID = 0;

    orb = cv::ORB::create(1000);

    file_name = data_path + "/" +"place.txt";
    fp = fopen(file_name.c_str(),"r");
    char words[1000];
    while(fgets(words,1000,fp)!=NULL){
        int imageID,x,y,yaw;
        char img_file_name[1000];
        sscanf(words,"%d %d %d %d %s",&imageID,&x,&y,&yaw,img_file_name);
        placeInformation plIn;
        plIn.id = imageID;
        plIn.x = x*0.1;
        plIn.y = y*0.1;
        plIn.yaw = yaw*0.01;
        plIn.img_file_name = img_file_name;
        placeInfo.push_back(plIn);
        printf("%d %d %d %d %s\n",imageID,x,y,yaw,img_file_name);
    }

    ros::Subscriber imageSubscriber = nh.subscribe<sensor_msgs::CompressedImage>("/camera/image_color/compressed", 100, imageCallback);
    ros::Subscriber gpsSubscriber = nh.subscribe<aloam_velodyne::dwdx>("/ros_dwdx", 100, getGPS);

    std::thread mapping_process{process};
    ros::spin();

    return 0;
}
