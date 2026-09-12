#include "rikibot_camera_cpp/astra_stream.hpp"
#include <cstring>

astra_status_t VideoFrameStream::StreamOpen(void) { 
    // signal(SIGINT, on_sigint); 
    astra_status_t stat;
    astra_streamsetconnection_t sensor;

    stat = astra_streamset_open("device/default", &sensor); 

    if (stat != ASTRA_STATUS_SUCCESS) 
        return stat; 
    sensor_ = sensor;

    return stat;
} 

astra_status_t VideoFrameStream::StreamStart(void) { 

    if (sensor_ == nullptr) {
        return ASTRA_STATUS_INVALID_PARAMETER;
    }

    astra_status_t stat;
    astra_reader_t reader;
    astra_colorstream_t color_stream;
    astra_depthstream_t depth_stream;

    stat = astra_reader_create(sensor_, &reader); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;
    reader_ = reader;

    stat = astra_reader_get_colorstream(reader_, &color_stream); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;
    color_stream_ = color_stream;

    stat = astra_reader_get_depthstream(reader_, &depth_stream); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;
    depth_stream_ = depth_stream;

    stat = astra_stream_start(color_stream_); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;

    stat = astra_stream_start(depth_stream_); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;

    stat = astra_reader_register_frame_ready_callback(reader_, &VideoFrameStream::FrameReadyCallback, this, &callback_id_); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;
    running_ = true;

    return ASTRA_STATUS_SUCCESS;
_return:
    if (callback_id_ != nullptr) {
        astra_reader_unregister_frame_ready_callback(&callback_id_); 
        callback_id_ = nullptr;
    }

    if (color_stream_ != nullptr) {
        astra_stream_stop(color_stream_);
        color_stream_ = nullptr;
    }

    if (depth_stream_ != nullptr) {
        astra_stream_stop(depth_stream_);
        depth_stream_ = nullptr;
    }

    if (reader_ != nullptr) {
        astra_reader_destroy(&reader_);
        reader_ = nullptr;
    }

    return stat;
}


// 处理 reader frame, 取出彩色和深度两路子帧, 两路子帧的帧号必定相同
void VideoFrameStream::FrameHandler(astra_reader_frame_t frame) 
{
    // 同一个 reader_frame 上取出两路子帧(彩色和深度共用这一个回调)
    astra_colorframe_t colorFrame = nullptr;
    astra_depthframe_t depthFrame = nullptr;
    astra_frame_get_colorframe(frame, &colorFrame);
    astra_frame_get_depthframe(frame, &depthFrame);

    astra_frame_index_t frameIndex; 
    astra_colorframe_get_frameindex(colorFrame, &frameIndex); 
    current_frame_id_ = frameIndex; 
    if (current_frame_id_ == last_frame_id_) 
        return; 
    last_frame_id_ = current_frame_id_; 

    /* color frame info */ 
    astra_image_metadata_t metadata; 
    astra_rgb_pixel_t*     colorData_rgb; 
    std::uint32_t colorByteLength; 
    astra_colorframe_get_data_rgb_ptr(colorFrame, &colorData_rgb, &colorByteLength); 
    astra_colorframe_get_metadata(colorFrame, &metadata); 

    /* depth frame info */ 
    astra_image_metadata_t depthMetadata; 
    std::int16_t*          depthData; 
    std::uint32_t depthByteLength; 
    astra_depthframe_get_data_ptr(depthFrame, &depthData, &depthByteLength); 
    astra_depthframe_get_metadata(depthFrame, &depthMetadata); 

    this->mtx.lock();

    // 更新最新颜色帧
    this->lastest_cr_frame_.w = metadata.width;
    this->lastest_cr_frame_.h = metadata.height;
    this->lastest_cr_frame_.stride = colorByteLength / metadata.height; //每行像素占用空间
    this->lastest_cr_frame_.pix_sz = lastest_cr_frame_.stride / metadata.width;
    this->lastest_cr_frame_.pix_fmt =
        static_cast<ColorFrame::pixel_format_t>(metadata.pixelFormat);
    this->lastest_cr_frame_.pix.resize(colorByteLength);
    std::memcpy(this->lastest_cr_frame_.pix.data(), colorData_rgb, colorByteLength);

    // 更新最新深度帧
    this->lastest_dph_frame_.w = depthMetadata.width;
    this->lastest_dph_frame_.h = depthMetadata.height;
    this->lastest_dph_frame_.stride = depthByteLength / depthMetadata.height; //每行像素占用空间
    this->lastest_dph_frame_.dph_sz = lastest_dph_frame_.stride / depthMetadata.width;
    this->lastest_dph_frame_.dph.resize(depthByteLength / sizeof(std::uint16_t));
    std::memcpy(this->lastest_dph_frame_.dph.data(), depthData, depthByteLength);

    this->mtx.unlock();
}

void VideoFrameStream::FrameReadyCallback(void* clientTag, astra_reader_t reader, astra_reader_frame_t frame) { 
    (void)reader;  
    auto self = static_cast<VideoFrameStream*>(clientTag); 
    if (self == nullptr || frame == nullptr) 
        return; 

    self->FrameHandler(frame); 
} 

bool VideoFrameStream::IsRunning(void) const { 
    return running_; 
} 

void VideoFrameStream::StreamStop(void) { 
    running_ = false;
    astra_reader_unregister_frame_ready_callback(&callback_id_); 
    astra_stream_stop(color_stream_);
    astra_stream_stop(depth_stream_);
    astra_reader_destroy(&reader_); 
} 

void VideoFrameStream::StreamClose(void) { 
    if (sensor_ != nullptr) {
        astra_streamset_close(&sensor_);
    }
}
