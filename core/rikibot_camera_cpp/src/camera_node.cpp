#include "rikibot_camera_cpp/astra_stream.hpp"
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node
{
    public:
        CameraNode() : Node("camera_node")
        {
            image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "camera/color/image_raw", 10);
        }

        // 把一帧彩色图像发布到 sensor_msgs/msg/Image 话题
        void PublishColorFrame(const ColorFrame& frame)
        {
            sensor_msgs::msg::Image msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "camera_color_optical_frame";
            msg.height = frame.h;
            msg.width = frame.w;
            msg.encoding = "rgb8"; // 直接写死
            msg.is_bigendian = 0;
            msg.step = frame.stride;
            msg.data = frame.pix;

            image_pub_->publish(msg);
        }

    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    public:
        VideoFrameStream stream_;
};

void* frame_handler(void* arg)
{
    VideoFrameStream * stream = (VideoFrameStream*)arg;
    std::uint32_t frame_id = -1;
    while (1) {
        std::lock_guard<std::mutex> lock(stream->mtx);
        if (frame_id == stream->GetCurFrameId()) {// 这里会空烧CPU，建议加一个同步机制（没时间加了）
            continue;
        }
        // update frame_id and handle the frame
        frame_id = stream->GetCurFrameId();

        std::cout << "frame_id: " << frame_id << std::endl;
        
    }
    return nullptr;
}

void* video_stream_update(void* arg)
{
    VideoFrameStream * stream = (VideoFrameStream*)arg;
    do { 
        astra_update(); 
    } while (stream->IsRunning()); 
    return nullptr;
}

int main(int argc, char* argv[]) 
{ 
    rclcpp::init(argc, argv);

    astra_initialize(); 

    CameraNode camera_node;

    // rclcpp::spin(camera_node.get_node_base()->get());





    VideoFrameStream stream; 
    stream.StreamOpen(); 
    stream.StreamStart(); 
    
    pthread_t ntid1, ntid2;
    if (pthread_create(
        &ntid1, 
        NULL, 
        video_stream_update, 
        &stream) != 0) {
        std::cerr << "pthread_create failed" << std::endl;
        goto exit;
    }
    if (pthread_create(
        &ntid2, 
        NULL, 
        frame_handler, 
        &stream) != 0) {
        std::cerr << "pthread_create failed" << std::endl;
        goto exit;
    }
    
exit:
    // 等待线程结束
    pthread_join(ntid1, nullptr);
    pthread_join(ntid2, nullptr);

    stream.StreamStop(); 
    stream.StreamClose(); 

    astra_terminate(); 

    rclcpp::shutdown();

    return 0; 
} 
