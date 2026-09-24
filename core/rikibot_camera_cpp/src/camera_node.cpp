#include "rikibot_camera_cpp/astra_stream.hpp"
#include <pthread.h>
#include <cmath>
#include <cstring>
#include <string>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node
{
    public:
        CameraNode() : Node("camera_node")
        {
            pub_ = image_transport::create_publisher(this, "camera/color/image_raw");
            depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", rclcpp::QoS(50));
            color_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/color/camera_info", rclcpp::QoS(50));
            depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("camera/depth/camera_info", rclcpp::QoS(50));
        }

        // 组装一份 camera_info: SLAM 靠它把像素坐标换算成三维坐标
        // 深度已对齐到彩色, 所以两份用同一组内参(即彩色相机视角下的内参)
        sensor_msgs::msg::CameraInfo MakeCameraInfo(const std::string& frame_id,
                                                   const rclcpp::Time& stamp) const
        {
            const CameraCalibration& cal = stream_.GetCalibration();

            sensor_msgs::msg::CameraInfo info;
            info.header.stamp = stamp;
            info.header.frame_id = frame_id;
            info.width  = cal.width;
            info.height = cal.height;

            // 畸变系数未知, 暂按无畸变处理(出厂一般已在硬件/固件侧校正)
            info.distortion_model = "plumb_bob";
            info.d = {0.0, 0.0, 0.0, 0.0, 0.0};

            info.k = {cal.fx, 0.0,    cal.cx,
                      0.0,    cal.fy, cal.cy,
                      0.0,    0.0,    1.0};

            info.r = {1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0};

            info.p = {cal.fx, 0.0,    cal.cx, 0.0,
                      0.0,    cal.fy, cal.cy, 0.0,
                      0.0,    0.0,    1.0,    0.0};

            return info;
        }

        // 把一帧彩色图像发布到 sensor_msgs/msg/Image 话题
        void PublishColorFrame(const ColorFrame& frame, const rclcpp::Time& stamp)
        {
            const std::string frame_id = "camera_color_optical_frame";

            sensor_msgs::msg::Image msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = frame_id;
            msg.height = frame.h;
            msg.width = frame.w;
            msg.encoding = "rgb8"; // 直接写死
            msg.is_bigendian = 0;
            msg.step = frame.stride;
            msg.data = frame.pix;

            pub_.publish(msg);
            color_info_pub_->publish(MakeCameraInfo(frame_id, stamp));
        }

        // 把一帧深度图像发布到 sensor_msgs/msg/Image 话题
        // 深度已对齐到彩色(D2C), 两张图共用彩色相机光心, 所以 frame_id 也用彩色光心
        void PublishDepthFrame(const DepthFrame& frame, const rclcpp::Time& stamp)
        {
            const std::string frame_id = "camera_color_optical_frame";

            sensor_msgs::msg::Image msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = frame_id;
            msg.height = frame.h;
            msg.width = frame.w;
            msg.encoding = "16UC1"; // 直接写死
            msg.is_bigendian = 0;
            msg.step = frame.stride;
            msg.data.resize(frame.dph.size() * sizeof(std::uint16_t));
            std::memcpy(msg.data.data(), frame.dph.data(), msg.data.size());

            depth_pub_->publish(msg);
            depth_info_pub_->publish(MakeCameraInfo(frame_id, stamp));
        }

        // 打印设备标定参数与深度对齐状态(只在启动时调用一次)
        void LogDeviceInfo()
        {
            const CameraCalibration& cal = stream_.GetCalibration();

            RCLCPP_INFO(this->get_logger(), "深度对齐到彩色(D2C): %s",
                        stream_.IsDepthRegistered() ? "已开启" : "未生效");

            // 出厂标定接口(astra_get_orbbec_camera_params): 部分固件返回成功但内容无效
            if (cal.factory_valid) {
                RCLCPP_INFO(this->get_logger(), "出厂标定 深度内参 fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                            cal.depth_intr[0], cal.depth_intr[1],
                            cal.depth_intr[2], cal.depth_intr[3]);
                RCLCPP_INFO(this->get_logger(), "出厂标定 彩色内参 fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
                            cal.color_intr[0], cal.color_intr[1],
                            cal.color_intr[2], cal.color_intr[3]);
                RCLCPP_INFO(this->get_logger(),
                            "出厂标定 深度畸变 k1=%.5f k2=%.5f p1=%.5f p2=%.5f k3=%.5f",
                            cal.depth_dist[0], cal.depth_dist[1], cal.depth_dist[2],
                            cal.depth_dist[3], cal.depth_dist[4]);
                RCLCPP_INFO(this->get_logger(),
                            "出厂标定 彩色畸变 k1=%.5f k2=%.5f p1=%.5f p2=%.5f k3=%.5f",
                            cal.color_dist[0], cal.color_dist[1], cal.color_dist[2],
                            cal.color_dist[3], cal.color_dist[4]);
                RCLCPP_INFO(this->get_logger(), "出厂标定 彩色->深度 平移 x=%.5f y=%.5f z=%.5f",
                            cal.r2l_t[0], cal.r2l_t[1], cal.r2l_t[2]);
                RCLCPP_INFO(this->get_logger(),
                            "出厂标定 彩色->深度 旋转 [%.5f %.5f %.5f | %.5f %.5f %.5f | %.5f %.5f %.5f]",
                            cal.r2l_r[0], cal.r2l_r[1], cal.r2l_r[2],
                            cal.r2l_r[3], cal.r2l_r[4], cal.r2l_r[5],
                            cal.r2l_r[6], cal.r2l_r[7], cal.r2l_r[8]);
            } else {
                RCLCPP_WARN(this->get_logger(),
                            "出厂标定接口没给出有效内参, 改用下面的换算缓存反推");
            }

            // 深度->世界坐标换算缓存: SDK 内部建点云时实际使用的参数
            if (cal.conv_valid) {
                RCLCPP_INFO(this->get_logger(),
                            "换算缓存: xz=%.8f yz=%.8f coeffX=%.3f coeffY=%.3f (分辨率 %dx%d)",
                            cal.conv_xz_factor, cal.conv_yz_factor,
                            cal.conv_coeff_x, cal.conv_coeff_y,
                            cal.conv_width, cal.conv_height);
            }

            // 最终写进 camera_info 的内参
            if (cal.intr_valid) {
                constexpr double kPi = 3.14159265358979323846;
                const double hfov = 2.0 * std::atan(cal.width / (2.0 * cal.fx)) * 180.0 / kPi;
                const double vfov = 2.0 * std::atan(cal.height / (2.0 * cal.fy)) * 180.0 / kPi;

                RCLCPP_INFO(this->get_logger(),
                            "写入 camera_info 的内参: fx=%.2f fy=%.2f cx=%.2f cy=%.2f "
                            "(%dx%d, hfov=%.2f度 vfov=%.2f度)",
                            cal.fx, cal.fy, cal.cx, cal.cy,
                            cal.width, cal.height, hfov, vfov);
                if (cal.cx_cy_assumed) {
                    RCLCPP_WARN(this->get_logger(),
                                "cx/cy 未标定, 取的是图像中心; 需要更准时用棋盘格标定彩色相机");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(),
                             "内参不可用, camera_info 里的 fx/fy/cx/cy 会是 0, SLAM 无法工作");
            }
        }

        // 只打印一次首帧信息, 用来核对实际分辨率/步长和代码里写死的编码是否一致
        void LogFrameShapeOnce(const ColorFrame& color, const DepthFrame& depth)
        {
            if (logged_frame_shape_) {
                return;
            }
            logged_frame_shape_ = true;

            RCLCPP_INFO(this->get_logger(), "彩色帧 %ux%u stride=%u 每像素字节=%u",
                        static_cast<unsigned>(color.w), static_cast<unsigned>(color.h),
                        static_cast<unsigned>(color.stride),
                        static_cast<unsigned>(color.pix_sz));
            RCLCPP_INFO(this->get_logger(), "深度帧 %ux%u stride=%u 每像素字节=%u",
                        static_cast<unsigned>(depth.w), static_cast<unsigned>(depth.h),
                        static_cast<unsigned>(depth.stride),
                        static_cast<unsigned>(depth.dph_sz));

            if (color.pix_sz != 3) {
                RCLCPP_WARN(this->get_logger(),
                            "彩色每像素不是 3 字节, 但发布时写死了 encoding=rgb8");
            }
            if (color.w != depth.w || color.h != depth.h) {
                RCLCPP_WARN(this->get_logger(),
                            "彩色与深度分辨率不一致: 深度没对齐到彩色");
            }
        }

    private:
        image_transport::Publisher pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
        bool logged_frame_shape_ = false;

    public:
        VideoFrameStream stream_;
};

void* frame_handler(void* arg)
{
    CameraNode * node = (CameraNode*)arg;
    VideoFrameStream & stream = node->stream_;
    std::uint32_t frame_id = -1;

    struct ColorFrame frame;
    struct DepthFrame depth_frame;
    while (rclcpp::ok()) {
        // 加锁只为了读当前帧号, 读完立刻解锁
        stream.mtx.lock();
        std::uint32_t cur_frame_id = stream.GetCurFrameId();

        if (!stream.HasFrame() || frame_id == cur_frame_id) {// 还没出图或帧号没变则空转（仍会空烧CPU，待加同步机制）
            stream.mtx.unlock();
            continue;
        }
        frame = stream.GetLatestColorFrame();
        depth_frame = stream.GetLatestDepthFrame();
        frame_id = cur_frame_id;

        stream.mtx.unlock();

        node->LogFrameShapeOnce(frame, depth_frame);

        // 发布
        rclcpp::Time stamp = node->now();
        node->PublishColorFrame(frame, stamp);
        node->PublishDepthFrame(depth_frame, stamp);
    }
    return nullptr;
}

void* video_stream_update(void* arg)
{
    VideoFrameStream * stream = (VideoFrameStream *)arg;
    do { 
        astra_update(); 
    } while (rclcpp::ok() && stream->IsRunning()); 
    return nullptr;
}

int main(int argc, char* argv[]) 
{ 
    rclcpp::init(argc, argv);

    astra_initialize(); 

    CameraNode camera_node;

    VideoFrameStream & stream = camera_node.stream_; 
    stream.StreamOpen(); 
    stream.StreamStart(); 

    camera_node.LogDeviceInfo();

    pthread_t ntid1 = 0, ntid2 = 0;
    bool t1_created = (pthread_create(
        &ntid1, 
        NULL, 
        video_stream_update, 
        &stream) == 0);
    bool t2_created = t1_created && (pthread_create(
        &ntid2, 
        NULL, 
        frame_handler, 
        &camera_node) == 0);
    if (!t1_created || !t2_created) {
        std::cerr << "pthread_create failed" << std::endl;
    } else {
        pthread_join(ntid1, nullptr);
        pthread_join(ntid2, nullptr);
    }

    stream.StreamStop(); 
    stream.StreamClose(); 

    astra_terminate(); 

    rclcpp::shutdown();

    return (t1_created && t2_created) ? 0 : 1; 
} 
