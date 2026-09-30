// 下行桥接: ROS 2 -> PX4 (Offboard 控制)
//
// 发布:
//   /fmu/in/offboard_control_mode   Offboard 心跳, 必须先于 setpoint 持续发送
//   /fmu/in/trajectory_setpoint     位置 + 偏航角 setpoint (NED)
//   /fmu/in/vehicle_command         切换 OFFBOARD 模式 / 解锁
// 订阅:
//   /fmu/out/vehicle_local_position 用于初始化 setpoint (避免上电瞬间飞向原点)
//   /fmu/out/vehicle_status         用于打印当前模式/解锁状态
//   ~/target_pose                   geometry_msgs/PoseStamped, ENU 坐标系下的目标位姿
//
// 安全设计:
//   - 首次收到 PX4 本地位置前, 不发送任何 setpoint;
//   - 初始 setpoint 取当前位置, 因此"刚切 Offboard"时不会产生位移;
//   - auto_arm 默认关闭, 只切模式不解锁, 需要显式打开参数才会发解锁指令;
//   - 未收到目标位姿时, setpoint 保持在当前位置。

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

namespace
{
constexpr double kPi = 3.14159265358979323846;

rclcpp::QoS px4Qos()
{
    return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
}

double normalizeAngle(double a)
{
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}
}  // namespace

class Px4OffboardControl : public rclcpp::Node
{
public:
    Px4OffboardControl()
    : rclcpp::Node("px4_offboard_control")
    {
        auto_offboard_ = this->declare_parameter<bool>("auto_offboard", true);
        auto_arm_ = this->declare_parameter<bool>("auto_arm", false);
        use_target_yaw_ = this->declare_parameter<bool>("use_target_yaw", false);

        offboard_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", px4Qos());
        setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", px4Qos());
        command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", px4Qos());

        target_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { onTargetPose(msg); });

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", px4Qos(),
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                local_pos_ = *msg;
                has_local_pos_ = true;
            });

        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", px4Qos(),
            [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) { onVehicleStatus(msg); });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), [this]() { onTimer(); });  // 50 Hz

        RCLCPP_INFO(
            get_logger(),
            "px4_offboard_control 已启动 (auto_offboard=%d, auto_arm=%d, use_target_yaw=%d)",
            auto_offboard_, auto_arm_, use_target_yaw_);
        RCLCPP_INFO(
            get_logger(), "向 %s 发布 ENU 目标位姿即可开始控制",
            target_sub_->get_topic_name());
    }

private:
    uint64_t nowUs() const
    {
        return static_cast<uint64_t>(this->now().nanoseconds() / 1000);
    }

    void onTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // ENU -> NED: 北=ENU.y, 东=ENU.x, 地=-ENU.z
        setpoint_[0] = static_cast<float>(msg->pose.position.y);
        setpoint_[1] = static_cast<float>(msg->pose.position.x);
        setpoint_[2] = static_cast<float>(-msg->pose.position.z);

        // ENU(FLU) 偏航角 -> NED 航向角 (以正北为 0, 顺时针为正)
        const auto & q = msg->pose.orientation;
        const double yaw_enu = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        setpoint_yaw_ = normalizeAngle(kPi / 2.0 - yaw_enu);

        if (!has_target_) {
            has_target_ = true;
            RCLCPP_INFO(get_logger(), "已收到第一个目标位姿, setpoint 开始跟随 target_pose");
        }
    }

    void onVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        // 只在状态变化时打印, 避免刷屏
        if (msg->nav_state == last_nav_state_ && msg->arming_state == last_arming_state_) return;
        last_nav_state_ = msg->nav_state;
        last_arming_state_ = msg->arming_state;
        RCLCPP_INFO(
            get_logger(), "飞控状态: nav_state=%u, arming_state=%u (OFFBOARD=%u, ARMED=%u)",
            msg->nav_state, msg->arming_state,
            px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD,
            px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
    }

    void onTimer() //使用offboard模式必须持续发送OffboardControlMode消息，一旦OffboardControlMode的发送频率掉到大约2Hz以下，PX4就会自动退出offboard模式
    {
        if (!has_local_pos_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 3000, "等待 PX4 本地位置, 暂不发送 setpoint");
            return;
        }

        // 初始 setpoint 取当前位置, 保证切换 Offboard 的瞬间不会产生位移
        if (!setpoint_initialized_) {
            setpoint_ = {local_pos_.x, local_pos_.y, local_pos_.z};
            setpoint_yaw_ = local_pos_.heading;
            setpoint_initialized_ = true;
            RCLCPP_INFO(
                get_logger(), "以当前位置初始化 setpoint: N=%.2f E=%.2f D=%.2f yaw=%.2f",
                setpoint_[0], setpoint_[1], setpoint_[2], setpoint_yaw_);
        }

        publishOffboardControlMode();
        publishSetpoint();

        // PX4 要求: 在切 Offboard 之前必须已经持续收到一段时间的 setpoint (约 1s)
        if (auto_offboard_ && !offboard_requested_ && counter_ > 50) {
            setOffboardMode();
            offboard_requested_ = true;
        }
        if (auto_arm_ && offboard_requested_ && !arm_requested_ && counter_ > 100) {
            arm();
            arm_requested_ = true;
        }

        ++counter_;
    }

    void publishOffboardControlMode()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.timestamp = nowUs();
        msg.position = true;   // 使用位置控制
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.actuator = false;
        offboard_mode_pub_->publish(msg);
    }

    void publishSetpoint()
    {
        const float nan = std::numeric_limits<float>::quiet_NaN();

        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp = nowUs();
        msg.position = {setpoint_[0], setpoint_[1], setpoint_[2]};
        // 未参与控制的维度置 NaN, 由 PX4 自行保持
        msg.velocity = {nan, nan, nan};
        msg.acceleration = {nan, nan, nan};
        msg.jerk = {nan, nan, nan};
        msg.yaw = use_target_yaw_ ? static_cast<float>(setpoint_yaw_) : nan;
        msg.yawspeed = nan;
        setpoint_pub_->publish(msg);
    }

    void setOffboardMode()
    {
        // param1 = 1 (CUSTOM_MODE_ENABLED), param2 = 6 (PX4_CUSTOM_MAIN_MODE_OFFBOARD)
        publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        RCLCPP_INFO(get_logger(), "已发送切换 OFFBOARD 模式指令");
    }

    void arm()
    {
        publishVehicleCommand(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
            static_cast<float>(px4_msgs::msg::VehicleCommand::ARMING_ACTION_ARM));
        RCLCPP_INFO(get_logger(), "已发送解锁指令");
    }

    void publishVehicleCommand(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp = nowUs();
        cmd.param1 = param1;
        cmd.param2 = param2;
        cmd.command = command;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.confirmation = 0;
        cmd.from_external = true;
        command_pub_->publish(cmd);
    }

    // 参数
    bool auto_offboard_;
    bool auto_arm_;
    bool use_target_yaw_;

    // 发布器 / 订阅器
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态
    px4_msgs::msg::VehicleLocalPosition local_pos_{};
    std::array<float, 3> setpoint_{0.0f, 0.0f, 0.0f};
    double setpoint_yaw_ = 0.0;
    bool has_local_pos_ = false;
    bool has_target_ = false;
    bool setpoint_initialized_ = false;
    bool offboard_requested_ = false;
    bool arm_requested_ = false;
    uint64_t counter_ = 0;
    uint8_t last_nav_state_ = 255;
    uint8_t last_arming_state_ = 255;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4OffboardControl>());
    rclcpp::shutdown();
    return 0;
}
