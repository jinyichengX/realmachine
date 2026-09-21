#include <iostream>
#include "KCF_Tracker.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include "kcftracker.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Rect selectRect;
Rect result;
bool bRenewROI = true;
bool bBeginKCF = false;
bool Joy_active = false;
Mat rgbimage;
Mat depthimage;
const int &ACTION_SPACE = 32;
cv::VideoCapture capture(8);

void ImageConverter::Reset() {
    bRenewROI = false;
    bBeginKCF = false;
    selectRect.x = 0;
    selectRect.y = 0;
    selectRect.width = 0;
    selectRect.height = 0;
    linear_speed = 0;
    rotation_speed = 0;
    enable_get_depth = false;
    this->linear_PID->reset();
    this->angular_PID->reset();
    vel_pub_->publish(geometry_msgs::msg::Twist());
}

void ImageConverter::Cancel() {
    bBeginKCF = false;
    linear_speed = 0;
    rotation_speed = 0;
    enable_get_depth = false;
    vel_pub_->publish(geometry_msgs::msg::Twist());
}

void ImageConverter::timer_callback() {
    capture >> rgbimage;
    sensor_msgs::msg::Image kcf_imagemsg;
    std_msgs::msg::Header _header;
    cv_bridge::CvImage _cv_bridge;
    _header.stamp = this->get_clock() -> now();
    _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, rgbimage);
    _cv_bridge.toImageMsg(kcf_imagemsg);
    rgb_pub_-> publish(kcf_imagemsg);

    if (bRenewROI && selectRect.width != 0) {
        tracker.init(selectRect, rgbimage);
        bBeginKCF = true;
        bRenewROI = false;
        enable_get_depth = false;
    }
    if (bBeginKCF) {
        result = tracker.update(rgbimage);
        rectangle(rgbimage, result, Scalar(0, 255, 255), 1, 8);
        circle(rgbimage, Point(result.x + result.width / 2, result.y + result.height / 2), 3, Scalar(0, 0, 255),-1);
    }
    else rectangle(rgbimage, selectRect, Scalar(255, 0, 0), 2, 8, 0);

    _header.stamp = this->get_clock() -> now();
    _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, rgbimage);
    _cv_bridge.toImageMsg(kcf_imagemsg);
    image_pub_-> publish(kcf_imagemsg);
}

// void ImageConverter::imageCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
//     cv_bridge::CvImagePtr cv_ptr;
//     try {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     }
//     catch (cv_bridge::Exception &e) {
//         std::cout<<"cv_bridge exception"<<std::endl;
//         return;
//     }
    
//     cv_ptr->image.copyTo(rgbimage);
//     if (bRenewROI && selectRect.width != 0) {
//         tracker.init(selectRect, rgbimage);
//         bBeginKCF = true;
//         bRenewROI = false;
//         enable_get_depth = false;
//     }
//     if (bBeginKCF) {
//         result = tracker.update(rgbimage);
//         rectangle(rgbimage, result, Scalar(0, 255, 255), 1, 8);
//         circle(rgbimage, Point(result.x + result.width / 2, result.y + result.height / 2), 3, Scalar(0, 0, 255),-1);
//     }

//     sensor_msgs::msg::Image kcf_imagemsg;
//     std_msgs::msg::Header _header;
//     cv_bridge::CvImage _cv_bridge;
//     _header.stamp = this->get_clock() -> now();
//     _cv_bridge = cv_bridge::CvImage(_header, sensor_msgs::image_encodings::BGR8, rgbimage);
//     _cv_bridge.toImageMsg(kcf_imagemsg);
//     image_pub_-> publish(kcf_imagemsg);
// }

void ImageConverter::depthCb(const std::shared_ptr<sensor_msgs::msg::Image> msg) {
    if (Joy_active) return;
    this->get_parameter<float>("linear_Kp",this->linear_KP);
    this->get_parameter<float>("linear_Ki",this->linear_KI);
    this->get_parameter<float>("linear_Kd",this->linear_KD);
    this->get_parameter<float>("angular_Kp",this->angular_KP);
    this->get_parameter<float>("angular_Ki",this->angular_KI);
    this->get_parameter<float>("angular_Kd",this->angular_KD);
    this->get_parameter<float>("minDistance",this->minDist);
    this->linear_PID = new PID(this->linear_KP, this->linear_KI, this->linear_KD);
    this->angular_PID = new PID(this->angular_KP, this->angular_KI, this->angular_KD);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv_ptr->image.copyTo(depthimage);
    }
    catch (cv_bridge::Exception &e) {
        std::cout<<"Could not convert from  to 'TYPE_32FC1'."<<std::endl;
    }
    if (enable_get_depth) {
        int center_x = (int)(result.x + result.width / 2);
        std::cout<<"center_x: "<<center_x<<std::endl;
        int center_y = (int)(result.y + result.height / 2);
        std::cout<<"center_y: "<<center_y<<std::endl;
        dist_val[0] = depthimage.at<float>(center_y - 5, center_x - 5);
        dist_val[1] = depthimage.at<float>(center_y - 5, center_x + 5);
        dist_val[2] = depthimage.at<float>(center_y + 5, center_x + 5);
        dist_val[3] = depthimage.at<float>(center_y + 5, center_x - 5);
        dist_val[4] = depthimage.at<float>(center_y, center_x);
        float distance = 0;
        int num_depth_points = 5;
        for (int i = 0; i < 5; i++) {
            if (dist_val[i] > 40 && dist_val[i] < 8000) distance += dist_val[i];
            else num_depth_points--;
        }
        if (num_depth_points == 0) distance = this->minDist;
        else distance /= num_depth_points;
        std::cout<<"distance: "<<distance<<std::endl;

        // std::cout<<"minDist: "<<minDist<<std::endl;
        distance /= 1000.0;
        linear_speed = -linear_PID->compute(this->minDist, distance);
        // std::cout<<"linear_speed: "<<linear_speed<<std::endl;
        if (linear_speed > 0.2) linear_speed = 0.2;
        if (linear_speed < -0.2) linear_speed = -0.2;
        
        rotation_speed = angular_PID->compute(320, center_x) / 1000.0;
        // std::cout<<"rotation_speed: "<<rotation_speed<<std::endl;
        if (rotation_speed > 0.2) rotation_speed = 0.2;
        if (rotation_speed < -0.2) rotation_speed = -0.2;

        if (abs(distance - this->minDist) < 0.03) linear_speed = 0;
        if (abs(center_x - 320.0) < 50) rotation_speed = 0;
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_speed * 1.0;
        twist.angular.z = rotation_speed * 1.0;
        std::cout<<"twist.linear.x: "<<twist.linear.x<<std::endl;
        std::cout<<"twist.angular.z: "<<twist.angular.z<<std::endl;
        vel_pub_->publish(twist);
    }
}

void ImageConverter::JoyCb(const std::shared_ptr<std_msgs::msg::Bool> msg) {
    Joy_active = msg->data;
}

void ImageConverter::MouseCb(const std::shared_ptr<geometry_msgs::msg::Polygon> msg) {
    if (bRenewROI) {
        selectRect.x = MIN((int)(msg->points[0].x), (int)(msg->points[1].x));
        selectRect.y = MIN((int)(msg->points[0].y), (int)(msg->points[1].y));
        selectRect.width = abs((int)(msg->points[0].x - msg->points[1].x));
        selectRect.height = abs((int)(msg->points[0].y - msg->points[1].y));
        selectRect &= Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
}

void ImageConverter::KeyCb(const std::shared_ptr<std_msgs::msg::Char> msg) {
    char action = msg->data;
    if (action == ACTION_SPACE) enable_get_depth = true;
    else if (action == 'i' || action == 'I')  bRenewROI = true;
    else if (action == 'r' || action == 'R')  this->Reset();
    else if (action == 'q' || action == 'Q')  this->Cancel();
}

int main(int argc,char **argv)
{
	rclcpp::init(argc, argv);
    std::cout<<"start"<<std::endl;
    rclcpp::spin(std::make_shared<ImageConverter>());
    return 0;
}
