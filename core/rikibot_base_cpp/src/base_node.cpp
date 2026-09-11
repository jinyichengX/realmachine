#include <iostream>
#include <vector>
#include <math.h>
#include <cstring>
#include <pthread.h>
#include <iostream>

#include "raw_serial.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>

class BaseNode : public LowLevelSerialPort, public rclcpp::Node {
    public:
        double x = 0, y = 0, th = 0;          /* 累计位姿：x(m) y(m) 朝向th(rad) */
        double vx0 = 0.0, vy0 = 0.0, vz0 = 0.0;    /* 上一帧的原始采样速度 */
        int    has_prev = 0;                  /* 是否已有上一帧 */
        double last_t = 0;                    /* 上一帧时刻(秒) */

        std::string odom_frame_;
        std::string base_frame_;
        tf2_ros::TransformBroadcaster odom_base_tf_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr        odom_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr         battery_pub_;
        nav_msgs::msg::Odometry   odom_msg_;

        // 自动选择时的候选路径，按顺序尝试
        static const std::vector<std::string> & PortCandidates()
        {
            static const std::vector<std::string> candidates = {
                // 宿主机安装 config/99-rikibase.rules 后由 udev 生成
                "/dev/rikibase",
                // udev 生成的稳定 by-id 路径，容器内同样可见
                "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
            };
            return candidates;
        }

        // 参数 serial_port 非空则直接使用；否则返回候选路径中第一个真实存在的
        std::string ResolvePort()
        {
            this->declare_parameter<std::string>("serial_port", "");
            const std::string configured = this->get_parameter("serial_port").as_string();
            if (!configured.empty()) return configured;

            for (const auto & p : PortCandidates()) {
                if (::access(p.c_str(), F_OK) == 0) return p;
            }
            return PortCandidates().front();
        }

        BaseNode()
            : LowLevelSerialPort(115200),
              rclcpp::Node("base_node"),
              odom_base_tf_(this) {
            const std::string port = ResolvePort();
            if (this->Open(port.c_str()) < 0) {
                std::string tried;
                for (const auto & p : PortCandidates()) tried += "\n    " + p;
                throw std::runtime_error(
                    "无法打开底盘串口 " + port +
                    "\n  已尝试的候选路径:" + tried +
                    "\n  查看当前设备: ls -l /dev/serial/by-id/"
                    "\n  或手动指定: ros2 run rikibot_base_cpp base_node "
                    "--ros-args -p serial_port:=/dev/ttyUSB1");
            }
            std::cout << "底盘串口已打开: " << port << std::endl;

            cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", rclcpp::QoS(10),
                [this](const geometry_msgs::msg::Twist::SharedPtr msg) { OnCmdVel(msg); });
            odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(50));
            battery_pub_ = this->create_publisher<std_msgs::msg::Float32>("battery", rclcpp::QoS(50));

            this->declare_parameter<std::string>("odom_frame", "odom");
            this->declare_parameter<std::string>("base_frame", "base_link");
            odom_frame_ = this->get_parameter("odom_frame").as_string();
            base_frame_ = this->get_parameter("base_frame").as_string();
        }

        ~BaseNode() = default;

        void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            SendCmdVel(msg->linear.x, msg->linear.y, msg->angular.z);
        }

        // 应答时间同步请求（话题编号 10）。
        // 实测底盘约11-12秒收不到时间同步应答就会触发保活机制并停发raw_vel raw_imu等话题
        void SendTimeSyncReply()
        {
            std::uint8_t frame[16];
            frame[0] = 0xFF; frame[1] = 0xFE;
            frame[2] = 0x08; frame[3] = 0x00;
            frame[4] = 0xF7;
            frame[5] = 0x0A; frame[6] = 0x00;

            unsigned int sec  = static_cast<unsigned int>(::time(nullptr));
            unsigned int nsec = 0;
            ::memcpy(frame + 7, &sec, 4);
            ::memcpy(frame + 11, &nsec, 4);

            unsigned int sum = 0;
            for (int i = 5; i < 15; ++i) sum += frame[i];
            frame[15] = static_cast<std::uint8_t>(0xFF - (sum & 0xFF));

            this->Send(frame, sizeof(frame));
        }

        // 发送一条 cmd_vel 速度指令帧（话题编号 101 共 56 字节）
        // 参数: vx = 前后线速度 m/s, vy = 左右线速度 m/s, wz = 旋转角速度 rad/s; 其余通道自动填 0
        // 保证至少20hz发一次！！！
        int SendCmdVel(double vx, double vy, double wz)
        {
            std::uint8_t frame[56];
            frame[0] = 0xFF; frame[1] = 0xFE;
            frame[2] = 0x30; frame[3] = 0x00;
            frame[4] = 0xCF;
            frame[5] = 0x65; frame[6] = 0x00;

            double vel[6] = {vx, vy, 0.0, 0.0, 0.0, wz};   // 麦轮底盘只需 x/y/z 三个自由度
            for (int i = 0; i < 6; ++i) {
                ::memcpy(frame + 7 + i * 8, &vel[i], sizeof(double));
            }

            std::uint32_t sum = 0;
            for (auto i = 5; i < 55; ++i) {
                sum += frame[i];
            }
            frame[55] = static_cast<std::uint8_t>(0xFF - (sum & 0xFF));

            return this->Send(frame, sizeof(frame));
        }

        // 发握手包
        void SendHandshake()
        {
            std::vector<std::uint8_t> handshake_package = {
                0xff, 0xfe, 0x00, 0x00, 0xff, 0x00, 0x00, 0xff};
            this->Send(handshake_package.data(), handshake_package.size());
        }

        // 读1字节（内部等串口可读字节凑够1）
        int ReadByte(std::uint8_t & c)
        {
            int avail = 0;
            int r = this->WaitRecvSize(1, &avail);
            if (r != ANS_OK) return r;
            if (this->Recv(&c, 1) != 1) return ANS_DEV_ERR;
            return ANS_OK;
        }

        // 从串口当前位置读完整一帧
        // 不假设流起点是帧头：逐字节滑动找FF FE；长度校验/数据校验任一不过即视为假帧头
        int ReadOneFrame(std::vector<std::uint8_t> & out_data)
        {
            std::uint8_t a, b;

            int r = this->ReadByte(a);
            if (r != ANS_OK) return r;

            while (1) {
                // 逐字节滑动窗口(a, b)，直到出现帧头FF FE
                while (1) {
                    r = this->ReadByte(b);
                    if (r != ANS_OK) return r;
                    if (a == 0xFF && b == 0xFE) break;
                    a = b;
                }

                std::uint8_t len3[3];
                int avail = 0;
                r = this->WaitRecvSize(3, &avail);
                if (r != ANS_OK) return r;
                if (this->Recv(len3, sizeof(len3)) != 3) return ANS_DEV_ERR;

                std::uint32_t len = len3[0] | (len3[1] << 8);   // 小端：数据区字节数
                if (len > 1024 ||
                    ((len3[0] + len3[1] + len3[2]) & 0xFF) != 0xFF) {
                    a = len3[2];
                    continue;
                }

                std::vector<std::uint8_t> rest(len + 3);
                r = this->WaitRecvSize(len + 3, &avail);
                if (r != ANS_OK) return r;
                if (this->Recv(rest.data(), rest.size()) != (int)rest.size())
                    return ANS_DEV_ERR;

                std::uint32_t sum = 0;
                for (size_t i = 0; i < len + 2; ++i) sum += rest[i];   // topic 2 + data len
                sum = (sum + rest[len + 2]) & 0xFF;
                if (sum != 0xFF) {
                    a = rest[len + 2];
                    continue;
                }

                // only return data area
                std::uint16_t topic = rest[0] | (rest[1] << 8);
                out_data.assign(rest.begin() + 2, rest.end() - 1);
                return static_cast<int>(topic);
            }
        }

        static double mono_now(void) {
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            return ts.tv_sec + ts.tv_nsec * 1e-9;
        }

        // 标定系数没参数化：麦轮轮径/打滑造成的系统误差，单靠积分公式消不掉，只能靠一个可调的 linear/angular scale 实车标定，目前包内写死为 1.0
        int OdomUpdate(double vx, double vy, double vz) {
            double now = mono_now();
            double dt  = now - last_t;
            last_t     = now;

            if (dt <= 0) return -1;
            if (dt > 0.5){
                has_prev = 0;
                return -1;
            }

            double mvx = vx, mvy = vy, mvz = vz;
            if (has_prev){
                mvx = (vx0 + vx) * 0.5;
                mvy = (vy0 + vy) * 0.5;
                mvz = (vz0 + vz) * 0.5;
            }

            vx0 = vx;
            vy0 = vy;
            vz0 = vz;
            has_prev = 1;

            double d  = mvz * dt;
            double ct = cos(th), st = sin(th);

            if (fabs(d) < 1e-6){       /* 近似直线，避免除零 */
                x += (mvx * ct - mvy * st) * dt;
                y += (mvx * st + mvy * ct) * dt;
            } else {
                double s = sin(d), c = cos(d);
                double bx = (mvx * s + mvy * (c-1)) / mvz;   /* 本体系位移 */
                double by = (mvx * (1 - c) + mvy * s) / mvz;
                x += bx * ct - by * st;                   /* 转到世界系 */
                y += bx * st + by * ct;
            }
            th += d;

            return 0;
        }

        void OdomPublish(void)
        {
            odom_msg_.header.stamp = this->now();
            odom_msg_.header.frame_id = odom_frame_;
            odom_msg_.child_frame_id = base_frame_;
            odom_msg_.pose.pose.position.x      = x;
            odom_msg_.pose.pose.position.y      = y;
            odom_msg_.pose.pose.position.z      = 0.0;
            odom_msg_.pose.pose.orientation.x   = 0.0;
            odom_msg_.pose.pose.orientation.y   = 0.0;
            odom_msg_.pose.pose.orientation.z   = sin(th / 2.0);
            odom_msg_.pose.pose.orientation.w   = cos(th / 2.0);
            odom_msg_.twist.twist.linear.x      = vx0;
            odom_msg_.twist.twist.linear.y      = vy0;
            odom_msg_.twist.twist.angular.z     = vz0;

            odom_pub_->publish(odom_msg_);
        }

        void Odom2BaseTransform(void)
        {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp         = odom_msg_.header.stamp;
            t.header.frame_id      = odom_msg_.header.frame_id;
            t.child_frame_id       = odom_msg_.child_frame_id;
            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = 0.0;
            t.transform.rotation.x    = 0.0;
            t.transform.rotation.y    = 0.0;
            t.transform.rotation.z    = sin(th / 2.0);
            t.transform.rotation.w    = cos(th / 2.0);
            odom_base_tf_.sendTransform(t);
        }

        void OdomHandler(double vx, double vy, double vz)
        {
            // calc odom
            OdomUpdate(vx, vy, vz);

            // publish topic /odom
            OdomPublish();

            // broadcast odom -> base_link TF
            Odom2BaseTransform();
        }

        void BatteryPublish(float voltage)
        {
            std_msgs::msg::Float32 msg;
            msg.data = voltage;
            battery_pub_->publish(msg);
        }
};

void * parse_thread(void * arg)
{
    std::shared_ptr<BaseNode> node = *static_cast<std::shared_ptr<BaseNode>*>(arg);
    //read and parse: 逐帧阻塞读取 (WaitRecvSize 按帧长度凑齐后再读)
    std::vector<std::uint8_t> data;
    while (1) {
        int topic = node->ReadOneFrame(data);
        if (topic < 0) {              // ANS_TIMEOUT / ANS_DEV_ERR
            std::cerr << "ReadOneFrame failed: " << topic << std::endl;
            break;
        }

        if (topic == 125 && data.size() >= 12) {    // handle raw_vel topic
            float vx, vy, wz;
            ::memcpy(&vx, &data[0], sizeof(float));
            ::memcpy(&vy, &data[4], sizeof(float));
            ::memcpy(&wz, &data[8], sizeof(float));
            node->OdomHandler(vx, vy, wz);          // 积分 + 发布 /odom
        } else if (topic == 127 && data.size() >= 4) {    // handle battery topic
            float voltage;
            ::memcpy(&voltage, &data[0], sizeof(float));
            node->BatteryPublish(voltage);
        } else if (topic == 126 && data.size() >= 72) {   // handle raw_imu topic
            // double ax, ay, az, gx, gy, gz, mx, my, mz;
            // ::memcpy(&ax, &data[0], 8);
            // ::memcpy(&ay, &data[8], 8);
            // ::memcpy(&az, &data[16], 8);
            // ::memcpy(&gx, &data[24], 8);
            // ::memcpy(&gy, &data[32], 8);
            // ::memcpy(&gz, &data[40], 8);
            // ::memcpy(&mx, &data[48], 8);
            // ::memcpy(&my, &data[56], 8);
            // ::memcpy(&mz, &data[64], 8);
            // std::cout << "imu: acc=(" << ax << "," << ay << "," << az
            //           << ") gyro=(" << gx << "," << gy << "," << gz
            //           << ") mag=(" << mx << "," << my << "," << mz << ")" << std::endl;
        } else if (topic == 10) {   // handle time sync request topic
            node->SendTimeSyncReply();
        }
    }
    return nullptr;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<BaseNode> node;
    try {
        node = std::make_shared<BaseNode>();
        node->SendHandshake();
    } catch (const std::exception & e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }

    std::cout << "BaseNode started" << std::endl;

    //create recv thread
    pthread_t ntid;
    if (pthread_create(&ntid, NULL, parse_thread, &node) != 0) {
        std::cerr << "pthread_create failed" << std::endl;
        return -1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    pthread_cancel(ntid);
    pthread_join(ntid, nullptr);

    return 0;
}
