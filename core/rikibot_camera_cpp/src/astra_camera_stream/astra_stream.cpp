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

    // 开启"深度对齐到彩色"(D2C): 必须在深度流 start 之前设置
    // 开启后深度图像素与彩色图一一对应, 是 RGB-D SLAM 的前提
    stat = astra_depthstream_set_registration(depth_stream_, true);
    if (stat != ASTRA_STATUS_SUCCESS) {
        std::cerr << "[camera] astra_depthstream_set_registration 调用失败, status="
                  << stat << std::endl;
    }

    stat = astra_depthstream_get_registration(depth_stream_, &depth_registered_);
    if (stat != ASTRA_STATUS_SUCCESS) {
        depth_registered_ = false;
        std::cerr << "[camera] astra_depthstream_get_registration 调用失败, status="
                  << stat << std::endl;
    }

    stat = astra_stream_start(color_stream_); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;

    stat = astra_stream_start(depth_stream_); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;

    stat = astra_reader_register_frame_ready_callback(reader_, &VideoFrameStream::FrameReadyCallback, this, &callback_id_); 
    if (stat != ASTRA_STATUS_SUCCESS) 
        goto _return;

    // 两路流都已启动, 此时再读一次, 补上深度->世界换算缓存
    QueryDeviceCalibration();

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

// 读取出厂标定参数(内参/外参/畸变)与深度->世界换算缓存
// 可重复调用: sensor_ 未就绪或 depth_stream_ 未建立时自动跳过对应部分
bool VideoFrameStream::QueryDeviceCalibration(void) 
{
    if (sensor_ == nullptr) {
        return false;
    }

    // (1) 出厂标定: 左右目内参 + 彩色->深度外参 + 畸变系数
    orbbec_camera_params raw;
    std::memset(&raw, 0, sizeof(raw));

    astra_status_t stat = astra_get_orbbec_camera_params(sensor_, &raw);
    if (stat == ASTRA_STATUS_SUCCESS &&
        raw.l_intr_p[0] > 0.0f && raw.l_intr_p[1] > 0.0f &&
        raw.r_intr_p[0] > 0.0f && raw.r_intr_p[1] > 0.0f) {
        calibration_.factory_valid = true;
        std::memcpy(calibration_.depth_intr, raw.l_intr_p, sizeof(raw.l_intr_p));
        std::memcpy(calibration_.color_intr, raw.r_intr_p, sizeof(raw.r_intr_p));
        std::memcpy(calibration_.r2l_r,      raw.r2l_r,    sizeof(raw.r2l_r));
        std::memcpy(calibration_.r2l_t,      raw.r2l_t,    sizeof(raw.r2l_t));
        std::memcpy(calibration_.depth_dist, raw.l_k,      sizeof(raw.l_k));
        std::memcpy(calibration_.color_dist, raw.r_k,      sizeof(raw.r_k));
    } else if (stat == ASTRA_STATUS_SUCCESS) {
        // 焦距必须为正; 这里为 NaN/0 说明固件不支持该接口, 只是没报错
        std::cerr << "[camera] astra_get_orbbec_camera_params 返回成功但焦距无效"
                  << " (l_intr_p[0]=" << raw.l_intr_p[0]
                  << ", r_intr_p[0]=" << raw.r_intr_p[0] << "), 该固件不支持此接口"
                  << std::endl;
    } else {
        std::cerr << "[camera] astra_get_orbbec_camera_params 失败, status="
                  << stat << std::endl;
    }

    // (2) 深度->世界坐标换算系数, 这是 SDK 内部建点云时实际使用的参数
    if (depth_stream_ != nullptr) {
        astra_conversion_cache_t cache;
        std::memset(&cache, 0, sizeof(cache));

        stat = astra_depthstream_get_depth_to_world_data(depth_stream_, &cache);
        if (stat == ASTRA_STATUS_SUCCESS) {
            calibration_.conv_valid    = true;
            calibration_.conv_xz_factor = cache.xzFactor;
            calibration_.conv_yz_factor = cache.yzFactor;
            calibration_.conv_coeff_x   = cache.coeffX;
            calibration_.conv_coeff_y   = cache.coeffY;
            calibration_.conv_width     = cache.resolutionX;
            calibration_.conv_height    = cache.resolutionY;
        } else {
            std::cerr << "[camera] astra_depthstream_get_depth_to_world_data 失败, status="
                      << stat << std::endl;
        }
    }

    // (3) 汇总出最终可用的内参: 出厂值优先, 拿不到就用换算缓存反推
    if (calibration_.factory_valid) {
        calibration_.intr_valid = true;
        calibration_.fx = calibration_.depth_intr[0];
        calibration_.fy = calibration_.depth_intr[1];
        calibration_.cx = calibration_.depth_intr[2];
        calibration_.cy = calibration_.depth_intr[3];
        calibration_.cx_cy_assumed = false;
    } else if (calibration_.conv_valid &&
               calibration_.conv_xz_factor > 0.0f && calibration_.conv_yz_factor > 0.0f) {
        // 换算缓存里 xzFactor = 宽/fx, yzFactor = 高/fy
        calibration_.intr_valid = true;
        calibration_.fx = calibration_.conv_width / calibration_.conv_xz_factor;
        calibration_.fy = calibration_.conv_height / calibration_.conv_yz_factor;
        // 换算缓存里没有主点, 暂取图像中心(经验值), 需要更准时用棋盘格标定彩色相机
        calibration_.cx = calibration_.conv_width / 2.0f;
        calibration_.cy = calibration_.conv_height / 2.0f;
        calibration_.cx_cy_assumed = true;
    }

    if (calibration_.conv_width > 0 && calibration_.conv_height > 0) {
        calibration_.width  = calibration_.conv_width;
        calibration_.height = calibration_.conv_height;
    }

    return calibration_.intr_valid;
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
    // 左右翻转填回(原因见文件末尾的说明):
    // 整像素翻转, 不翻每个像素内部的字节序, 否则颜色的 RGB 通道会调换
    {
        const std::size_t row_bytes = colorByteLength / metadata.height;
        const std::size_t pix_bytes = row_bytes / metadata.width;
        for (std::uint32_t r = 0; r < metadata.height; ++r) {
            const std::uint8_t* src =
                reinterpret_cast<const std::uint8_t*>(colorData_rgb) +
                static_cast<std::size_t>(r) * row_bytes;
            std::uint8_t* dst = this->lastest_cr_frame_.pix.data() +
                                static_cast<std::size_t>(r) * row_bytes;
            for (std::uint32_t c = 0; c < metadata.width; ++c) {
                std::memcpy(dst + c * pix_bytes,
                            src + (metadata.width - 1 - c) * pix_bytes,
                            pix_bytes);
            }
        }
    }

    // 更新最新深度帧
    this->lastest_dph_frame_.w = depthMetadata.width;
    this->lastest_dph_frame_.h = depthMetadata.height;
    this->lastest_dph_frame_.stride = depthByteLength / depthMetadata.height; //每行像素占用空间
    this->lastest_dph_frame_.dph_sz = lastest_dph_frame_.stride / depthMetadata.width;
    this->lastest_dph_frame_.dph.resize(depthByteLength / sizeof(std::uint16_t));
    // 左右翻转填回, 与彩色帧用同一套翻转, 保证两图像素仍然一一对应
    for (std::uint32_t r = 0; r < depthMetadata.height; ++r) {
        const std::int16_t* src = depthData + static_cast<std::size_t>(r) * depthMetadata.width;
        std::uint16_t* dst = this->lastest_dph_frame_.dph.data() +
                             static_cast<std::size_t>(r) * depthMetadata.width;
        for (std::uint32_t c = 0; c < depthMetadata.width; ++c) {
            dst[c] = static_cast<std::uint16_t>(src[depthMetadata.width - 1 - c]);
        }
    }

    has_frame_ = true;

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

//---------------------------------------------------------------------------
// 关于取流时做左右翻转(FrameHandler 里那两处)
//
// 实测现象: 本机相机的彩色图和深度图相对现实都是左右镜像的
// (站在相机位置朝前看, 现实中在左边的东西出现在图像的右边)。
// 注意不是"彩色和深度互相镜像" —— 两路是同样的镜像方向, 所以点云的左右被整体翻转,
// 现实中在左侧的墙会被重建到相机右侧, 拼出来的地图必然错乱。
//
// 为什么不用 SDK 的镜像开关(astra_imagestream_set_mirroring):
//   该参数挂在流上, 而深度对齐(D2C)是按流的镜像状态一并处理的。
//   去动它有可能让已经验证通过的"深度与彩色逐像素对应"失效。
//   所以在 SDK 之后、进入我们自己的数据结构时按行翻转, 两路用同一套逻辑,
//   既保证彩色与深度继续一一对应, 又不影响 SDK 内部的对齐计算。
//
// 翻转时的两个要点:
//   1. 只翻转"像素"的顺序, 不能翻转每个像素内部的字节序, 否则 RGB 会变成 BGR;
//   2. 彩色和深度必须同时翻, 否则两路的像素就不再对应了。
//
// 如果以后换用不镜像的设备, 把这两个循环改回 memcpy 即可。
//---------------------------------------------------------------------------
