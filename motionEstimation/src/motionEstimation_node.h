#ifndef MOTIONESTIMATION_NODE_H_
#define MOTIONESTIMATION_NODE_H_
#include "ros/ros.h"
#include <std_msgs/String.h>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "sensor_msgs/Image.h"
#include "motionEstimation/Utility.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>

struct Camera
{
    cv::Mat left;
    cv::Mat right;
};

class motionEstimation_node
{
public:
    motionEstimation_node();
    cv::Mat imageCallback(sensor_msgs::ImagePtr &left, sensor_msgs::ImagePtr &right);
   // void imageCalc(sensor_msgs::ImagePtr &img_left, sensor_msgs::ImagePtr &img_right);
    ~motionEstimation_node();
protected:
int mode;
std::vector<std::string> filenames_left, filenames_right;
cv::Mat K_L, distCoeff_L, K_R, distCoeff_R;
cv::Mat E_LR, F_LR, R_LR, T_LR;
cv::Mat Q;
cv::Mat KInv_L, KInv_R;
cv::Mat P_LR, rvec_LR;
cv::Mat P_0;
cv::Mat R_0, T_0;
cv::Mat new_img_l, old_img_l, old_img_r, new_img_r;
ros::NodeHandle node_;
image_transport::Subscriber image_sub;
image_transport::Subscriber image_r;
std::string c_left;
std::string c_right;
cv::Mat Q;
cv::Mat currentPos_ES_L;
cv::Mat currentPos_ES_R;
cv::Mat currentPos_ES_mean;
cv::Mat currentPos_PnP_L;
cv::Mat currentPos_PnP_R;
cv::Mat currentPos_Stereo;
int image_sub_count;
std::vector<cv::Point2f> points_L1_temp, points_R1_temp;
std::vector<cv::Point2f> features;
std::vector<cv::Point2f> points_L1, points_R1, points_L2, points_R2;
std::vector<cv::Point2f> inliersHorizontal_L1, inliersHorizontal_R1, inliersHorizontal_L2, inliersHorizontal_R2;
cv::Mat F_L;
bool foundF_L;
std::vector<cv::Point2f> inliersF_L1, inliersF_L2;
cv::Mat F_R;
bool foundF_R;
std::vector<cv::Point2f> inliersF_R1, inliersF_R2;
cv::Mat T_E_L, R_E_L, T_E_R, R_E_R;
std::vector<cv::Point3f> pointCloud_1, pointCloud_2;
float u_L1, u_R1;
cv::Mat P_L, P_R;
std::vector<cv::Point3f> stereoCloud, nearestPoints;
int skipFrameNumber;
ros::Publisher posePublisher_;
};


#endif
