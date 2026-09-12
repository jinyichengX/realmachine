#include "rikibot_camera_cpp/astra_stream.hpp"
#include <pthread.h>
#include <cstring>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node
{
    public:
        CameraNode() : Node("camera_node")
        {
            pub_ = image_transport::create_publisher(this, "camera/color/image_raw");
            depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/depth/image_raw", rclcpp::QoS(50));
        }

        // 把一帧彩色图像发布到 sensor_msgs/msg/Image 话题
        void PublishColorFrame(const ColorFrame& frame, const rclcpp::Time& stamp)
        {
            sensor_msgs::msg::Image msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = "camera_color_optical_frame";
            msg.height = frame.h;
            msg.width = frame.w;
            msg.encoding = "rgb8"; // 直接写死
            msg.is_bigendian = 0;
            msg.step = frame.stride;
            msg.data = frame.pix;

            pub_.publish(msg);
        }

        // 把一帧深度图像发布到 sensor_msgs/msg/Image 话题
        void PublishDepthFrame(const DepthFrame& frame, const rclcpp::Time& stamp)
        {
            sensor_msgs::msg::Image msg;
            msg.header.stamp = stamp;
            msg.header.frame_id = "camera_depth_optical_frame";
            msg.height = frame.h;
            msg.width = frame.w;
            msg.encoding = "16UC1"; // 直接写死
            msg.is_bigendian = 0;
            msg.step = frame.stride;
            msg.data.resize(frame.dph.size() * sizeof(std::uint16_t));
            std::memcpy(msg.data.data(), frame.dph.data(), msg.data.size());

            depth_pub_->publish(msg);
        }

    private:
        image_transport::Publisher pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

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

        if (frame_id == cur_frame_id) {// 这里会空烧CPU，建议加一个同步机制（没时间加了）
            stream.mtx.unlock();
            continue;
        }
        frame = stream.GetLatestColorFrame();
        depth_frame = stream.GetLatestDepthFrame();
        frame_id = cur_frame_id;

        stream.mtx.unlock();

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
