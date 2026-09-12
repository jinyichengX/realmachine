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

    std::uint16_t  w;
    std::uint16_t  h;
    std::uint32_t  stride;
    std::uint8_t   pix_sz;
    pixel_format_t pix_fmt;
    std::vector<std::uint8_t> pix;
};

struct DepthFrame {
    std::uint16_t  w;
    std::uint16_t  h;
    std::uint32_t  stride;
    std::uint8_t   dph_sz;
    std::vector<std::uint16_t> dph; //直接16位写死
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

        std::uint32_t GetCurFrameId() {return current_frame_id_;}

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
    public:
        std::mutex                  mtx;
    private:
        struct ColorFrame           lastest_cr_frame_; //永远存图像最新帧
        struct DepthFrame           lastest_dph_frame_; //永远存深度最新帧

        bool                        running_ = false;
}; 
