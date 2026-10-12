// 台架测试用: 伪造外部视觉位置 (仅限拆桨或系留, 禁止带桨运行)
//
// 目的:
//   持续向飞控发送"外部视觉位置 = 原点"的假数据, 让 EKF2 得到水平位置估计,
//   从而 vehicle_local_position 的 xy_valid 变为 1, 使位置版 Offboard 能够解锁。
//
// 危险:
//   飞控会一直认为自己在原点且静止。一旦飞行器真实移动, 位置控制会完全错误。
//   因此本节点默认拒绝启动, 必须显式传 confirm_bench_test:=true。
//
// 发布:
//   /fmu/in/vehicle_visual_odometry   px4_msgs/msg/VehicleOdometry, 默认 50 Hz

#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace
{
rclcpp::QoS px4Qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}
}  // namespace

class Px4FakeEv : public rclcpp::Node
{
public:
    Px4FakeEv()
    : rclcpp::Node("px4_fake_ev")
    {
        confirm_bench_test_ = this->declare_parameter<bool>("confirm_bench_test", false);
        rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);

        if (!confirm_bench_test_) {
            RCLCPP_ERROR(
                get_logger(),
                "本节点会伪造位置数据, 飞控会误以为飞行器静止在原点。"
                "确认已拆桨或系留后, 加上 confirm_bench_test:=true 重新运行。");
            return;
        }

        pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", px4Qos());

        const auto period_us = static_cast<int64_t>(1e6 / rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::microseconds(period_us), [this]() { publishFakeOdometry(); });

        RCLCPP_WARN(
            get_logger(), "!!! 正在发送伪造位置 (原点, %.0f Hz), 仅供拆桨/系留台架测试 !!!",
            rate_hz_);
    }

private:
    void publishFakeOdometry()
    {
        px4_msgs::msg::VehicleOdometry msg{};
        const uint64_t stamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
        msg.timestamp = stamp;
        msg.timestamp_sample = stamp;

        // 位置与姿态: 固定在原点, 姿态与 NED 对齐 (w, x, y, z)
        msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        msg.position = {0.0f, 0.0f, 0.5f};
        msg.q = {1.0f, 0.0f, 0.0f, 0.0f};

        msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.angular_velocity = {0.0f, 0.0f, 0.0f};

        msg.position_variance = {kPositionVariance, kPositionVariance, kPositionVariance};
        msg.orientation_variance = {
            kOrientationVariance, kOrientationVariance, kOrientationVariance};
        msg.velocity_variance = {kVelocityVariance, kVelocityVariance, kVelocityVariance};

        msg.reset_counter = 0;
        msg.quality = 100;

        pub_->publish(msg);
    }

    static constexpr float kPositionVariance = 0.01f;
    static constexpr float kOrientationVariance = 0.01f;
    static constexpr float kVelocityVariance = 0.01f;

    bool confirm_bench_test_;
    double rate_hz_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4FakeEv>());
    rclcpp::shutdown();
    return 0;
}
