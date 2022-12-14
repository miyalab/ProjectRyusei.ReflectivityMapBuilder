//-----------------------------
// include
//-----------------------------
// STL
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <filesystem>

// OpenCV
#include <opencv2/opencv.hpp>

// ROS2
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <ros_robocore_interfaces/msg/robot_state_msg.hpp>

#include "cvt_functions/quaternion_to_euler.hpp"
#include "filesystem/log_load_functions_v2.hpp"
#include "point_rotational_transform/point_rotational_transform.hpp"

#define DEBUG

namespace rs = project_ryusei;
using geometry_msgs::msg::Point32;
using sensor_msgs::msg::PointCloud;
using ros_robocore_interfaces::msg::RobotStateMsg;

constexpr double unit          = 0.10;
constexpr double ref_max       = 0.20;
constexpr double ref_min       = 0.02;
constexpr double r_min         = 5.0;
constexpr double r_max         = 15.0;
constexpr double z_max         = 0.2;
constexpr char mapImgPath[]    = "/home/kmiyauchi/share/map/tsukuba/tsukuba_1106/tsukuba_kansou_hokan_v2_denoised.png";
constexpr char logPath[]       = "/home/kmiyauchi/share/sensor_20221106_141716/";
// constexpr char logPath[]       = "/home/kmiyauchi/share/sensor_20221106_141716/";
constexpr char robotStateDir[] = "mercury_blue/mercury_state.csv";
constexpr char locationDir[]   = "location/location_comp.csv";
constexpr char pointsDir[]     = "pandar_40/";

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv)
{
    // cv::Mat mapImg(10001, 10001, CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat mapImg = cv::imread(mapImgPath);
    if(mapImg.empty()) mapImg = cv::Mat(10001, 10001, CV_8UC3, cv::Scalar(0,0,0));
    cv::cvtColor(mapImg, mapImg, cv::COLOR_BGR2HSV_FULL);

    RobotStateMsg robotStateMsg;
    std::ifstream robotStateIfs((std::string)logPath + robotStateDir);
    std::string line; std::getline(robotStateIfs, line);
    Pose location;
    std::ifstream locationIfs((std::string)logPath + locationDir);
    std::getline(locationIfs, line);

    rs::PointRotationalTransform transform;
    transform.setOffset(0.0, -0.1, 0.4);

    for(int i=0; rs::loadMercuryState(robotStateIfs, &robotStateMsg) && rs::loadPose(locationIfs, &location); i++){
        double roll, pitch, location_yaw, imu_yaw;
        
        rs::quaternionToEulerAngle(location.orientation, &roll, &pitch, &location_yaw);
        rs::quaternionToEulerAngle(robotStateMsg.imu.orientation, &roll, &pitch, &imu_yaw);
        // transform.setAngle(roll, pitch, location_yaw);
        transform.setAngle(0, 0, location_yaw);
        
        char stamp[16];
        PointCloud points;
        std::snprintf(stamp, sizeof(stamp), "%06d.csv", i*2+1);
        // std::snprintf(stamp, sizeof(stamp), "%06d.csv", i);
        std::ifstream pointCloudIfs((std::string)logPath + pointsDir + stamp);
        rs::loadPointCloud(pointCloudIfs, &points);

#ifdef DEBUG
        cv::Mat pointsImg(500,500,CV_8UC3, cv::Scalar(0,0,0));
#else
        std::cout << i << ": " << points.points.size() << std::endl;
#endif
        for(int j=0, size=points.points.size(); j<size; j++){
            points.points[j] = transform.transPoint(points.points[j]);

            if(points.channels[1].values[j] > ref_max) continue;
            if(points.channels[1].values[j] < ref_min) continue;
            if(points.channels[0].values[j] < r_min) continue;
            if(points.channels[0].values[j] > r_max) continue;
            if(points.points[j].z > z_max) continue;

#ifdef DEBUG
            int px = pointsImg.cols/2 - points.points[j].y/unit;
            int py = pointsImg.rows/2 - points.points[j].x/unit;
            if(0<= px && px < pointsImg.cols){
                if(0<= py && py < pointsImg.rows){
                    pointsImg.at<cv::Vec3b>(py, px)[0] = 170.0*(1.0-(points.channels[1].values[j]-ref_min)/(ref_max-ref_min));
                    pointsImg.at<cv::Vec3b>(py, px)[1] = 255;
                    pointsImg.at<cv::Vec3b>(py, px)[2] = 255;
                }
            }
#endif
            
            const int &x = mapImg.cols/2 - (points.points[j].y + location.position.y)/unit;
            const int &y = mapImg.rows/2 - (points.points[j].x + location.position.x)/unit;

            if(x<0 || mapImg.cols<=x) continue;
            if(y<0 || mapImg.rows<=y) continue;
            if(mapImg.at<cv::Vec3b>(y,x)[1]!=0 || mapImg.at<cv::Vec3b>(y,x)[2]!=255){
                mapImg.at<cv::Vec3b>(y,x)[0] = std::max((double)mapImg.at<cv::Vec3b>(y,x)[0], 170.0*(1.0-(points.channels[1].values[j]-ref_min)/(ref_max-ref_min)));
                mapImg.at<cv::Vec3b>(y,x)[1] = 255;
                mapImg.at<cv::Vec3b>(y,x)[2] = 255;
            }
        }

#ifdef DEBUG
        cv::Mat debugImg;
        cv::cvtColor(mapImg, debugImg, cv::COLOR_HSV2BGR_FULL);
        cv::cvtColor(pointsImg, pointsImg, cv::COLOR_HSV2BGR_FULL);
        cv::Mat localMap(
            debugImg,
            cv::Rect(
                debugImg.cols/2 - location.position.y/unit - 250,
                debugImg.rows/2 - location.position.x/unit - 250,
                500, 500)
        );
        cv::resize(debugImg, debugImg, cv::Size(1001, 1001));
        cv::imshow("map", debugImg);
        cv::imshow("point", pointsImg);
        cv::imshow("localMap", localMap);
        if(cv::waitKey(1) == 'q') break;
#endif
    }

    cv::cvtColor(mapImg, mapImg, cv::COLOR_HSV2BGR_FULL);
    cv::imwrite("reflectivity_map.png", mapImg);

    cv::resize(mapImg, mapImg, cv::Size(1001, 1001));
    cv::imshow("map", mapImg);
    std::cout << "finish!" << std::endl; 
    cv::waitKey(0);
}

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------