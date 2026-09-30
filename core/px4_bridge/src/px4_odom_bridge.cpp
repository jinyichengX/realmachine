// 上行桥接: PX4 -> ROS 2
//
// 订阅:
//   /fmu/out/vehicle_local_position   本地位置/速度 (NED)
//   /fmu/out/vehicle_attitude         姿态四元数 (FRD -> NED)
// 发布:
//   ~/odom        nav_msgs/Odometry
//   TF: odom_frame -> base_frame
//
// 坐标系说明:
//   PX4 使用 NED(北-东-地) / FRD(前-右-下), ROS 使用 ENU(东-北-天) / FLU(前-左-上),
//   因此位置、速度需要换轴, 姿态四元数需要做坐标系旋转。
//
// QoS 说明:
//   PX4 侧 (uXRCE-DDS) 全部以 best-effort 发布, 这里必须同样使用 best-effort,
//   否则一帧都收不到。

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

namespace
{
rclcpp::QoS px4Qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

// NED -> ENU: 东=北轴的交换, 天=-地
void nedToEnu(double n, double e, double d, double & x, double & y, double & z)
{
    x = e;   // ENU x = East
    y = n;   // ENU y = North
    z = -d;  // ENU z = Up
}

// 姿态四元数由 PX4(FRD->NED) 转到 ROS(FLU->ENU)
// 推导: R_enu_flu = M * R_ned_frd * M', 其中 M 为 NED->ENU, M' 为 FLU->FRD
// 对应四元数 q_enu_flu = q_m ⊗ q_px4 ⊗ q_m', q_m=(0, 1/√2, 1/√2, 0), q_m'=(0, 1, 0, 0)
void attitudeNedToEnu(const std::array<float, 4> & q, double out[4])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double s = 1.0 / std::sqrt(2.0);
    out[0] = -(w + z) * s;
    out[1] = -(x + y) * s;
    out[2] = (y - x) * s;
    out[3] = (z - w) * s;
}
}  // namespace

class Px4OdomBridge : public rclcpp::Node
{
public:
    Px4OdomBridge()
    : rclcpp::Node("px4_odom_bridge"),
      tf_broadcaster_(this)
    {
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 20);

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", px4Qos(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                local_pos_ = *msg;
                has_local_pos_ = true;
            });

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", px4Qos(),
            [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                attitude_ = *msg;
                has_attitude_ = true;
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), [this]() { onTimer(); });  // 50 Hz

        RCLCPP_INFO(
            get_logger(), "px4_odom_bridge 已启动: /fmu/out/* -> %s + TF(%s -> %s)",
            odom_pub_->get_topic_name(), odom_frame_.c_str(), base_frame_.c_str());
    }

private:
    void onTimer()
    {
        if (!has_local_pos_ || !has_attitude_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000, "等待 PX4 数据 (local_position / attitude)...");
            return;
        }

        // 没有有效的水平/垂直位置(例如无 GPS、无 VIO)时不发布 odom, 避免给导航栈喂垃圾数据
        if (!local_pos_.xy_valid || !local_pos_.z_valid) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000,
                "PX4 本地位置无效 (xy_valid=%d, z_valid=%d), 暂不发布 odom; "
                "请确认 EKF2 有有效的位置来源 (GPS / 视觉里程计 / 动捕)",
                local_pos_.xy_valid, local_pos_.z_valid);
            return;
        }

        const rclcpp::Time stamp = this->now();

        double px, py, pz;
        nedToEnu(local_pos_.x, local_pos_.y, local_pos_.z, px, py, pz);
        double vx, vy, vz;
        nedToEnu(local_pos_.vx, local_pos_.vy, local_pos_.vz, vx, vy, vz);

        double q[4];
        attitudeNedToEnu(attitude_.q, q);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = px;
        odom.pose.pose.position.y = py;
        odom.pose.pose.position.z = pz;
        odom.pose.pose.orientation.w = q[0];
        odom.pose.pose.orientation.x = q[1];
        odom.pose.pose.orientation.y = q[2];
        odom.pose.pose.orientation.z = q[3];

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = vz;

        // PX4 给出的是位置/速度的标准差, 这里换算成协方差对角项
        odom.pose.covariance[0] = local_pos_.eph * local_pos_.eph;
        odom.pose.covariance[7] = local_pos_.eph * local_pos_.eph;
        odom.pose.covariance[14] = local_pos_.epv * local_pos_.epv;
        odom.twist.covariance[0] = local_pos_.evh * local_pos_.evh;
        odom.twist.covariance[7] = local_pos_.evh * local_pos_.evh;
        odom.twist.covariance[14] = local_pos_.evv * local_pos_.evv;

        odom_pub_->publish(odom);

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.transform.translation.x = px;
        tf.transform.translation.y = py;
        tf.transform.translation.z = pz;
        tf.transform.rotation = odom.pose.pose.orientation;
        tf_broadcaster_.sendTransform(tf);
    }

    std::string odom_frame_;
    std::string base_frame_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    px4_msgs::msg::VehicleLocalPosition local_pos_{};
    px4_msgs::msg::VehicleAttitude attitude_{};
    bool has_local_pos_ = false;
    bool has_attitude_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OdomBridge>());
    rclcpp::shutdown();
    return 0;
}
