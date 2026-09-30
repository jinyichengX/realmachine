// 独立小工具: 订阅 /fmu/out/vehicle_attitude, 把姿态四元数转成欧拉角并实时打印。
//
// 说明:
//   VehicleAttitude.q 是"从机体系(FRD)旋转到 NED"的四元数, 顺序为 [w, x, y, z]。
//   因此解出的 roll / pitch / yaw 都基于 NED 参考系:
//     roll  = 绕北向轴    (右滚为正)
//     pitch = 绕东向轴    (抬头为正)
//     yaw   = 绕地向轴    (机头相对"北"的航向, 顺时针为正)
//
// 订阅:
//   /fmu/out/vehicle_attitude   px4_msgs/msg/VehicleAttitude
//
// 打印:
//   固定频率(默认 10 Hz)输出一行, 单位: 度

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_attitude.hpp>

namespace
{
constexpr double kRadToDeg = 57.29577951308232;

rclcpp::QoS px4Qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}
}  // namespace

class Px4AttitudePrint : public rclcpp::Node
{
public:
    Px4AttitudePrint()
    : rclcpp::Node("px4_attitude_print")
    {
        print_rate_hz_ = this->declare_parameter<double>("print_rate_hz", 10.0);

        sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", px4Qos(),
            [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
                q_ = msg->q;
                has_data_ = true;
            });

        const auto period_us = static_cast<int64_t>(1e6 / print_rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(period_us), [this]() { printEuler(); });

        RCLCPP_INFO(
            get_logger(), "订阅 /fmu/out/vehicle_attitude, 以 %.0f Hz 打印欧拉角 (单位: 度)",
            print_rate_hz_);
    }

private:
    void printEuler()
    {
        if (!has_data_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000, "等待 /fmu/out/vehicle_attitude ...");
            return;
        }

        // 四元数 [w, x, y, z] -> 欧拉角 (NED 参考系)
        const double w = q_[0];
        const double x = q_[1];
        const double y = q_[2];
        const double z = q_[3];

        const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        // 夹紧一下, 避免浮点误差让 asin 的入参越出 [-1, 1]
        const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
        const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

        std::cout << "roll=" << std::setw(8) << std::fixed << std::setprecision(2)
                  << roll * kRadToDeg
                  << "  pitch=" << std::setw(8) << pitch * kRadToDeg
                  << "  yaw=" << std::setw(8) << yaw * kRadToDeg
                  << std::endl;
    }

    double print_rate_hz_;
    std::array<float, 4> q_{1.0f, 0.0f, 0.0f, 0.0f};
    bool has_data_ = false;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4AttitudePrint>());
    rclcpp::shutdown();
    return 0;
}
