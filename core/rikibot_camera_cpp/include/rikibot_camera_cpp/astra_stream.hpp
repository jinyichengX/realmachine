#pragma once

#include <astra/capi/astra.h> 
#include <stdio.h> 
#include <signal.h> 
#include <iostream> 
#include <cstdint>
#include <mutex>
#include <vector>

struct ColorFrame {
    typedef enum {
        ASTRA_PIXEL_FORMAT_UNKNOWN = 0,
        // color layouts
        ASTRA_PIXEL_FORMAT_RGB888 = 200,
        ASTRA_PIXEL_FORMAT_YUV422 = 201,
        ASTRA_PIXEL_FORMAT_YUYV = 202,
        ASTRA_PIXEL_FORMAT_RGBA = 203,
        ASTRA_PIXEL_FORMAT_NV21 = 204,
        ASTRA_PIXEL_FORMAT_GRAY8 = 300,
        ASTRA_PIXEL_FORMAT_GRAY16 = 301,
        ASTRA_PIXEL_FORMAT_POINT = 400,
    } pixel_format_t;

    std::uint16_t  w = 0;
    std::uint16_t  h = 0;
    std::uint32_t  stride = 0;
    std::uint8_t   pix_sz = 0;
    pixel_format_t pix_fmt = ASTRA_PIXEL_FORMAT_UNKNOWN;
    std::vector<std::uint8_t> pix;
};

struct DepthFrame {
    std::uint16_t  w = 0;
    std::uint16_t  h = 0;
    std::uint32_t  stride = 0;
    std::uint8_t   dph_sz = 0;
    std::vector<std::uint16_t> dph; //直接16位写死
};

// 设备标定参数、深度对齐状态与最终可用内参
struct CameraCalibration {
    bool  factory_valid = false;    // 出厂标定接口是否给出了有效值

    float depth_intr[4] = {0};      // 出厂值: 深度/IR 相机内参 [fx, fy, cx, cy]
    float color_intr[4] = {0};      // 出厂值: 彩色相机内参 [fx, fy, cx, cy]
    float r2l_r[9]      = {0};      // 出厂值: 彩色 -> 深度的旋转矩阵(行优先)
    float r2l_t[3]      = {0};      // 出厂值: 彩色 -> 深度的平移向量
    float depth_dist[5] = {0};      // 出厂值: 深度畸变 [k1, k2, p1, p2, k3]
    float color_dist[5] = {0};      // 出厂值: 彩色畸变 [k1, k2, p1, p2, k3]

    bool  conv_valid    = false;    // 是否读到深度->世界坐标的换算缓存
    float conv_xz_factor = 0.0f;    // SDK 内部建点云时实际使用的换算系数
    float conv_yz_factor = 0.0f;
    float conv_coeff_x   = 0.0f;
    float conv_coeff_y   = 0.0f;
    int   conv_width     = 0;       // 换算缓存对应的分辨率
    int   conv_height    = 0;

    // 最终用于 camera_info 的内参: 出厂值优先, 出厂接口无效时由换算缓存反推
    bool  intr_valid    = false;
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    bool  cx_cy_assumed = false;    // cx/cy 未标定, 暂取图像中心(经验值)
    int   width  = 0;
    int   height = 0;
};

class VideoFrameStream { 
    public:   
        VideoFrameStream()  = default; 
        ~VideoFrameStream() = default; 

        astra_status_t StreamOpen(void);
        astra_status_t StreamStart(void);
        void StreamStop(void);
        void StreamClose(void);
        bool IsRunning(void) const;

        // 读取设备出厂标定参数与深度->世界换算缓存(可重复调用, 未就绪的部分自动跳过)
        bool QueryDeviceCalibration(void);
        const CameraCalibration& GetCalibration(void) const { return calibration_; }
        bool IsDepthRegistered(void) const { return depth_registered_; }

        std::uint32_t GetCurFrameId() {return current_frame_id_;}
        bool HasFrame(void) const {return has_frame_;}

        ColorFrame GetLatestColorFrame() {
            return lastest_cr_frame_;
        }

        DepthFrame GetLatestDepthFrame() {
            return lastest_dph_frame_;
        }
    private: 
        static void FrameReadyCallback(void* clientTag, astra_reader_t reader, astra_reader_frame_t frame);
        void FrameHandler(astra_reader_frame_t frame);

        astra_streamsetconnection_t sensor_ = nullptr; 
        astra_reader_t              reader_ = nullptr; 
        astra_colorstream_t         color_stream_ = nullptr; 
        astra_depthstream_t         depth_stream_ = nullptr; 
        astra_reader_callback_id_t  callback_id_ = nullptr; 

        astra_frame_index_t         current_frame_id_ = 0; 
        astra_frame_index_t         last_frame_id_ = -1; 
        bool                        has_frame_ = false;  //是否已收到过至少一帧
    public:
        std::mutex                  mtx;
    private:
        struct ColorFrame           lastest_cr_frame_; //永远存图像最新帧
        struct DepthFrame           lastest_dph_frame_; //永远存深度最新帧

        CameraCalibration           calibration_;      //设备出厂标定参数
        bool                        depth_registered_ = false; //深度是否已对齐到彩色
        bool                        running_ = false;
}; 
