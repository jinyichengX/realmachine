//
// Created by yahboom on 2021/7/30.
//

#ifndef TRANSBOT_ASTRA_KCF_TRACKER_H
#define TRANSBOT_ASTRA_KCF_TRACKER_H

#include <iostream>
#include <algorithm>
#include <dirent.h>

#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include "kcftracker.h"
#include "PID.h"
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/char.hpp"
#include <time.h>

using namespace std;
using namespace cv;
using std::placeholders::_1;

class ImageConverter :public rclcpp::Node{
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr mouse_sub_;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr key_sub_;
    rclcpp::TimerBase::SharedPtr timer;
    
public:
    ImageConverter():Node("image_converter")
    {            
        this->declare_parameter<float>("linear_Kp",3.0);
        this->declare_parameter<float>("linear_Ki",0.0);
        this->declare_parameter<float>("linear_Kd",2.0);
        this->declare_parameter<float>("angular_Kp",0.5);
        this->declare_parameter<float>("angular_Ki",0.0);
        this->declare_parameter<float>("angular_Kd",2.0);
        this->declare_parameter<float>("minDistance",1.0);
        
        //sub
        // image_sub_=this->create_subscription<sensor_msgs::msg::Image>("/camera/color/image_raw",1,std::bind(&ImageConverter::imageCb,this,_1));
        depth_sub_=this->create_subscription<sensor_msgs::msg::Image>("/camera/depth/image_raw",1,std::bind(&ImageConverter::depthCb,this,_1));
        Joy_sub_=this->create_subscription<std_msgs::msg::Bool>("JoyState",1,std::bind(&ImageConverter::JoyCb,this,_1));
        mouse_sub_=this->create_subscription<geometry_msgs::msg::Polygon>("mouse_pos",1,std::bind(&ImageConverter::MouseCb,this,_1));
        key_sub_=this->create_subscription<std_msgs::msg::Char>("key_value",1,std::bind(&ImageConverter::KeyCb,this,_1));
        //pub
        image_pub_=this->create_publisher<sensor_msgs::msg::Image>("/KCF_image",1);
        vel_pub_ =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);

        rgb_pub_ =this->create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw",1);
        timer=this->create_wall_timer(20ms, std::bind(&ImageConverter::timer_callback,this));
    }

    float linear_KP = 3.0;
    float linear_KI = 0.0;
    float linear_KD = 2.0;
    float angular_KP = 0.5;
    float angular_KI = 0.0;
    float angular_KD = 2.0;
    float minDist = 1.0;

    PID *linear_PID = new PID(linear_KP, linear_KI, linear_KD);
    PID *angular_PID = new PID(angular_KP, angular_KI, angular_KD);
    
    float linear_speed = 0;
    float rotation_speed = 0;
    bool enable_get_depth = false;
    float dist_val[5];
    int center_x;
    KCFTracker tracker;


    void Reset();

    void Cancel();

    // void imageCb(const std::shared_ptr<sensor_msgs::msg::Image> msg);

    void depthCb(const std::shared_ptr<sensor_msgs::msg::Image> msg);

    void JoyCb(const std::shared_ptr<std_msgs::msg::Bool> msg);

    void MouseCb(const std::shared_ptr<geometry_msgs::msg::Polygon> msg);

    void KeyCb(const std::shared_ptr<std_msgs::msg::Char> msg);

    void timer_callback();

};


#endif //TRANSBOT_ASTRA_KCF_TRACKER_H
