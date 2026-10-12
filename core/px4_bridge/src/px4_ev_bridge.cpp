// EV 桥: LIO/SLAM 里程计 -> PX4 外部视觉里程计
//
// 作用:
//   把 LIO 输出的 nav_msgs/Odometry(ROS 惯例: ENU 世界系 + FLU 机体系),
//   转换成 PX4 的 px4_msgs/VehicleOdometry(NED 世界系 + FRD 机体系),
//   发布到 /fmu/in/vehicle_visual_odometry, 由 EKF2 作为外部位置观测融合。
//
// 订阅:
//   odom   nav_msgs/Odometry      话题名由参数 odom_topic 指定, 默认 /Odometry
// 发布:
//   /fmu/in/vehicle_visual_odometry   px4_msgs/VehicleOdometry, 与 odom 同频
//
// 坐标变换:
//   位置:  N =  y_enu      E =  x_enu      D = -z_enu
//   姿态:  q_ned_frd = q_ENU2NED ⊗ q_enu_flu ⊗ q_FLU2FRD
//          q_ENU2NED = (0, √2/2, √2/2, 0)   绕 ENU 的 (1,1,0) 轴转 180°
//          q_FLU2FRD = (0, 1, 0, 0)         绕机体系 X 轴转 180°
//
// 设计说明:
//   - 速度暂不发布(置 NaN 表示"该维度不参与控制");
//     需要时先打开 EKF2_EV_CTRL 的 velocity 位, 再在这里补上;
//   - odom 断流时桥停止发布, 让飞控自己触发失效保护(不做"保持最后值");
//   - timestamp 用本机时钟, 与已验证可行的 px4_fake_ev 保持一致;
//     LIO 自身的处理延迟请用 EKF2_EV_DELAY 补偿。

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace
{
// √2/2
constexpr double kSqrtHalf = 0.7071067811865476;

rclcpp::QoS px4Qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

// 四元数乘法, 分量顺序 (w, x, y, z)
std::array<float, 4> quatMul(const std::array<float, 4> & a, const std::array<float, 4> & b)
{
    return {
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]
    };
}
}  // namespace

class Px4EvBridge : public rclcpp::Node
{
public:
    Px4EvBridge()
    : rclcpp::Node("px4_ev_bridge")
    {
        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/Odometry");
        position_variance_ = this->declare_parameter<double>("position_variance", 0.01);
        orientation_variance_ = this->declare_parameter<double>("orientation_variance", 0.01);
        use_frd_frame_ = this->declare_parameter<bool>("use_frd_frame", true);

        pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(
            "/fmu/in/vehicle_visual_odometry", px4Qos());

        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::QoS(rclcpp::KeepLast(10)),
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });

        // 每秒检查一次 odom 是否断流
        watchdog_ = this->create_wall_timer(
            std::chrono::seconds(1), [this]() { checkOdomAlive(); });

        RCLCPP_INFO(
            get_logger(), "EV 桥已启动: 订阅 %s, 发布 /fmu/in/vehicle_visual_odometry",
            odom_topic_.c_str());
        RCLCPP_INFO(
            get_logger(), "pose_frame=%s, position_variance=%.4f, orientation_variance=%.4f",
            use_frd_frame_ ? "FRD" : "NED", position_variance_, orientation_variance_);
    }

private:
    uint64_t nowUs() const
    {
        return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_odom_time_ = this->now();
        if (!has_odom_) {
            has_odom_ = true;
            RCLCPP_INFO(get_logger(), "已收到第一条 odom, 开始向飞控转发");
        }
        ++odom_count_;

        const auto & p = msg->pose.pose.position;      // ENU: x=东, y=北, z=上
        const auto & q = msg->pose.pose.orientation;   // FLU 机体系 -> ENU 世界系

        px4_msgs::msg::VehicleOdometry out{};
        out.timestamp = nowUs();
        out.timestamp_sample = out.timestamp;

        // 位置: ENU -> NED
        out.pose_frame = use_frd_frame_
            ? px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD //本质上还是NED
            : px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
        out.position = {
            static_cast<float>(p.y),
            static_cast<float>(p.x),
            static_cast<float>(-p.z)
        };

        // 姿态: q_enu_flu(ROS) -> q_ned_frd(PX4)
        const std::array<float, 4> q_ros = {
            static_cast<float>(q.w), static_cast<float>(q.x),
            static_cast<float>(q.y), static_cast<float>(q.z)
        };
        const std::array<float, 4> q_enu2ned = {
            0.0f, static_cast<float>(kSqrtHalf), static_cast<float>(kSqrtHalf), 0.0f
        };
        const std::array<float, 4> q_flu2frd = {0.0f, 1.0f, 0.0f, 0.0f};

        std::array<float, 4> q_px4 = quatMul(quatMul(q_enu2ned, q_ros), q_flu2frd);
        normalizeQuat(q_px4);
        out.q = q_px4;

        // 速度暂不参与控制, 置 NaN
        const float nan = std::numeric_limits<float>::quiet_NaN();
        out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
        out.velocity = {nan, nan, nan};
        out.angular_velocity = {nan, nan, nan};

        out.position_variance = {
            static_cast<float>(position_variance_),
            static_cast<float>(position_variance_),
            static_cast<float>(position_variance_)
        };
        out.orientation_variance = {
            static_cast<float>(orientation_variance_),
            static_cast<float>(orientation_variance_),
            static_cast<float>(orientation_variance_)
        };
        out.velocity_variance = {
            static_cast<float>(position_variance_),
            static_cast<float>(position_variance_),
            static_cast<float>(position_variance_)
        };

        out.reset_counter = 0;
        out.quality = 100;

        pub_->publish(out);
    }

    // 归一化, 并把符号统一到 w >= 0(四元数双覆盖)
    static void normalizeQuat(std::array<float, 4> & q)
    {
        const double n = std::sqrt(
            static_cast<double>(q[0]) * q[0] + static_cast<double>(q[1]) * q[1] +
            static_cast<double>(q[2]) * q[2] + static_cast<double>(q[3]) * q[3]);
        if (n < 1e-9) {
            q = {1.0f, 0.0f, 0.0f, 0.0f};
            return;
        }
        for (auto & v : q) {
            v = static_cast<float>(v / n);
        }
        if (q[0] < 0.0f) {
            for (auto & v : q) {
                v = -v;
            }
        }
    }

    void checkOdomAlive()
    {
        if (!has_odom_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000, "还没收到 %s, 未向飞控发布任何数据",
                odom_topic_.c_str());
            return;
        }
        const double dt = (this->now() - last_odom_time_).seconds();
        if (dt > 1.0) {
            RCLCPP_WARN(
                get_logger(),
                "odom 已断流 %.1f 秒 (累计收到 %lu 帧); 飞控将在约 5 秒后触发失效保护",
                dt, static_cast<unsigned long>(odom_count_));
        }
    }

    std::string odom_topic_;
    double position_variance_;
    double orientation_variance_;
    bool use_frd_frame_;

    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr watchdog_;

    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    bool has_odom_ = false;
    uint64_t odom_count_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4EvBridge>());
    rclcpp::shutdown();
    return 0;
}
