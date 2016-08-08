#ifndef LCM_DEPTH_PROVIDER_H
#define LCM_DEPTH_PROVIDER_H

// DART header
#include <dart/depth_sources/depth_source.h>
#include <dart/util/mirrored_memory.h>

// LCM and message header
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include "lcm_provider_base.hpp"

// threading
#include <thread>
#include <mutex>
#include <atomic>
#include <boost/thread.hpp>

// stream input/output
#include <iostream>

// zlib de/-compression
#include <zlib.h>

// set optional filter distance in meter to remove all points beyond this point
//#define FILTER_DIST 0.5

/**
 * @brief jpeg_decompress decompress jpeg data to bitmap
 * @param jpg_buffer pointer to jpeg data
 * @param jpg_size size of jpeg data buffer (e.g. the length of compressed data)
 * @param bmp_buffer pointer to bitmap buffer that will contain the decompressed data, this buffer needs to be allocated with the expected image size (width * height * pixel-size)
 * @return true on success
 * @return false on failure
 */
bool jpeg_decompress(uint8_t *jpg_buffer, unsigned long jpg_size, uint8_t *bmp_buffer) {
    struct jpeg_error_mgr jerr;
    struct jpeg_decompress_struct cinfo;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);

    jpeg_mem_src(&cinfo, jpg_buffer, jpg_size);

    // check jpeg header
    int ret = jpeg_read_header(&cinfo, TRUE);
    if(ret!=1) {
        std::cerr<<"Data is not in jpeg format!"<<std::endl;
        return false;
    }

    // read and decompress data
    jpeg_start_decompress(&cinfo);
    while(cinfo.output_scanline < cinfo.output_height) {
        unsigned char *buffer_array[1];
        buffer_array[0] = bmp_buffer + (cinfo.output_scanline*cinfo.output_width*cinfo.output_components);
        jpeg_read_scanlines(&cinfo, buffer_array, 1);
    }

    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    return true;
}

namespace dart {

////////////////////////////////////////////////////////////////////////////////
/// \brief The StereoCameraParameter struct
///

struct StereoCameraParameter {
    float2 focal_length;    // f_x, f_y in pxl
    float2 camera_center;   // c_x, c_y in pxl
    float baseline;         // b in meter
    uint64_t width;         // image width in pxl
    uint64_t height;        // image height in pxl

    /**
     * @brief subpixel_resolution resolution of stored disparity resolution in pixel,
     * e.g. disparity_in_pixel = 1/subpixel_resolution * disparity_stored
     */
    float subpixel_resolution = 1.0;

    /**
     * @brief depth_resolution resolution of stored depth values in meter,
     * e.g. depth_in_meter = 1/resolution * depth_stored
     */
    float depth_resolution = 1.0;
};

////////////////////////////////////////////////////////////////////////////////
/// LCM_DepthSource class definition
///
template <typename DepthType, typename ColorType>
class LCM_DepthSource : public LCM_CommonBase, public dart::DepthSource<DepthType,ColorType> {
private:

#ifdef CUDA_BUILD
    /**
     * @brief _depthData pointer to class for managing memory representation at host system and GPU memory
     */
    dart::MirroredVector<DepthType> * _depthData;
#else
    /**
     * @brief _depthData pointer to regular array storing depth values
     */jpeg_read_scanlines
    DepthType * _depthData;
#endif // CUDA_BUILD

    ColorType * _colorData;

    /**
     * @brief _depthTime timestamp (seconds since the epoch) of depth data
     */
    uint64_t _depthTime;

    uint64_t _colourTime;

    /**
     * @brief _cam_param internal structure to store camera parameters
     */
    StereoCameraParameter _cam_param;

    /**
     * @brief _ScaleToMeters optional scale that is multiplied with internally stored values to obtain measurement in meter. It is applied just before the backprojection.
     */
    float _ScaleToMeters;

    /**
     * @brief _mutex synchronize access to data
     */
    mutable boost::shared_mutex _mutex;

public:
    /**
     * @brief LCM_DepthSource
     * @param param structure contatining all stereo camera parameters
     * @param scale optional scale parameter that is multiplied with depth value to scale to meters
     */
    LCM_DepthSource(const StereoCameraParameter &param, const float scale = 1.0);

    ~LCM_DepthSource();

    /**
     * @brief setFrame
     * @param frame
     */
    void setFrame(const uint frame);

    /**
     * @brief advance
     */
    void advance();

    /**
     * @brief hasRadialDistortionParams
     * @return true if sensor has radial distortion
     */
    bool hasRadialDistortionParams() const;

#ifdef CUDA_BUILD
    /**
     * @brief getDepth
     * @return pointer to array containing depth values
     */
    const DepthType * getDepth() const {
        std::lock_guard<boost::shared_mutex> lck(_mutex);
        return _depthData->hostPtr();
    }

    /**
     * @brief getDeviceDepth
     * @return pointer to array containing depth values
     */
    const DepthType * getDeviceDepth() const {
        std::lock_guard<boost::shared_mutex> lck(_mutex);
        return _depthData->devicePtr();
    }
#else
    /**
     * @brief getDepth
     * @return pointer to array containing depth values
     */
    const DepthType * getDepth() const {
        std::lock_guard<boost::shared_mutex> lck(_mutex);
        return _depthData;
    }

    /**
     * @brief getDeviceDepth
     * @return pointer to array containing depth values
     */
    const DepthType * getDeviceDepth() const { return 0; }
#endif // CUDA_BUILD

    const ColorType * getColor() const { return _colorData; }

    ColorLayout getColorLayout() const { return LAYOUT_RGB; }

    /**
     * @brief getScaleToMeters
     * @return scale paramter that is multiplied with internal depth values to obtain values in meters
     */
    float getScaleToMeters() const { return _ScaleToMeters; }

    /**
     * @brief getDepthTime
     * @return current timestamp of sensor reading as epoch
     */
    uint64_t getDepthTime() const {
        std::lock_guard<boost::shared_mutex> lck(_mutex);
        return _depthTime;
    }

    uint64_t getColorTime() const {
        std::lock_guard<boost::shared_mutex> lck(_mutex);
        return _colourTime;
    }

    /**
     * @brief subscribe_images initializing LCM subscriber to images_t topic
     *
     * This method sets up the subscription for the requested channel. By default, the handling of messages (and thus advance()) will block. Set threading to true to wait for incomming messages in a dedicated thread. Alternatively to wait for incomming messages in a single thread, a timeout value van be set. This value is not used if threading is true.
     *
     * @param channel topic name of stereo camera, e.g. "CAMERA"
     * @return true on success
     * @return false on failure
     */
    bool subscribe_images(const std::string &img_channel);

    /**
     * @brief imgHandle callback function that is called each time a new message arrives.
     * Extract depth image from LCM message, uncompress if necessary and store their content.
     * @param rbuf
     * @param channel
     * @param msg
     */
    void imgHandle(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
                   const bot_core::images_t* msg);

    /**
     * @brief disparity_to_depth helper function to convert LCM's disparity to depth values using the stereo camera parameters. Disparity values are transfered to depth values in memory, e.g. no new array will be created and the provided disparit values will directly be changes to depth values in memory.
     * @param disparity_img array containing disparity values, will contain depth values after function call
     */
    void disparity_to_depth(DepthType *disparity_img);
};

////////////////////////////////////////////////////////////////////////////////
/// Implementation of LCM_DepthSource
///

template <typename DepthType, typename ColorType>
LCM_DepthSource<DepthType,ColorType>::LCM_DepthSource(const StereoCameraParameter &param, const float scale) {

    // depth source properties
    this->_isLive = true; // no way to control LCM playback from here
    this->_hasColor = true;
    this->_hasTimestamps = true;
    this->_frame = 0; // initialize first frame

    // camera properties
    _cam_param = param;
    this->_focalLength = param.focal_length;
    this->_principalPoint = param.camera_center;
    this->_depthWidth = param.width;
    this->_depthHeight = param.height;
    this->_colorWidth = param.width;
    this->_colorHeight = param.height;

    // set scale to depth resolution if scale has not been set
    if(scale==1.0 && _cam_param.depth_resolution!=1.0)
        _ScaleToMeters = _cam_param.depth_resolution;
    else
        _ScaleToMeters = scale;

    // allocate memory for depth image
#ifdef CUDA_BUILD
    _depthData = new dart::MirroredVector<DepthType>(this->_depthWidth*this->_depthHeight);
#else
    _depthData = new DepthType[this->_depthWidth*this->_depthHeight];
#endif // CUDA_BUILD

    _colorData = new ColorType[this->_colorWidth*this->_colorHeight];
}

template <typename DepthType, typename ColorType>
LCM_DepthSource<DepthType,ColorType>::~LCM_DepthSource() {
#ifdef CUDA_BUILD
    delete _depthData;
#else
    delete [] _depthData;
#endif // CUDA_BUILD

    delete [] _colorData;
}

template <typename DepthType, typename ColorType>
void LCM_DepthSource<DepthType,ColorType>::setFrame(const uint frame) {
    // nothing to do, we cannot control LCM playback from here
    if(this->_isLive) return;
}

template <typename DepthType, typename ColorType>
void LCM_DepthSource<DepthType,ColorType>::advance() { }

template <typename DepthType, typename ColorType>
bool LCM_DepthSource<DepthType,ColorType>::hasRadialDistortionParams() const {
    return false;
}

template <typename DepthType, typename ColorType>
bool LCM_DepthSource<DepthType,ColorType>::subscribe_images(const std::string &img_channel) {
    if(!good()) {
        return false;
    }

    subscribe(img_channel, &LCM_DepthSource<DepthType, ColorType>::imgHandle, this);

    return true;
}

template <typename DepthType, typename ColorType>
void LCM_DepthSource<DepthType,ColorType>::imgHandle(const lcm::ReceiveBuffer* rbuf, const std::string& channel,
               const bot_core::images_t* msg) {

    _mutex.lock();
    this->_frame++;
    _mutex.unlock();

    // read data
    const int64_t n_img = msg->n_images;
    const std::vector<int16_t> image_types = msg->image_types;

    // pointer to constant values of raw data, we are not going to change the image
    const uint8_t * raw_data = NULL;
    const uint8_t * colour_data = NULL;
    bool convert;

    // go over all images
    for(unsigned int i=0; i<n_img; i++) {
        const int16_t img_type = image_types[i];
        // number of bytes of raw data
        const int32_t image_size_raw = msg->images[i].size;
        // real expected image size (width*height*byte_per_pixel)
        uint64_t image_size_real = msg->images[i].row_stride * msg->images[i].height;

        // process image data
        switch(img_type) {
        // raw disparity image data
        case bot_core::images_t::DISPARITY:
        case bot_core::images_t::DEPTH_MM:
            raw_data = msg->images[i].data.data();
            convert = (img_type==bot_core::images_t::DISPARITY);
            break;

        // zlib compressed disparity or depth image data
        case bot_core::images_t::DISPARITY_ZIPPED:
        case bot_core::images_t::DEPTH_MM_ZIPPED:
        {
            // decompress data
            const uint8_t * raw_data_compressed = msg->images[i].data.data();
            // allocate memory for uncompressed image
            raw_data = new uint8_t[image_size_real];
            // uncompress and check for errors
            const int zip_res = uncompress((Bytef *)raw_data, &image_size_real, (Bytef *)raw_data_compressed, image_size_raw);
            if(zip_res!=Z_OK) {
                std::cerr<<"something went wrong in decompressing data (err: "<<zip_res<<")"<<std::endl;
                switch(zip_res) {
                case Z_MEM_ERROR:
                    std::cerr<<"(Z_MEM_ERROR) not enough memory"<<std::endl;
                    break;
                case Z_BUF_ERROR:
                    std::cerr<<"(Z_BUF_ERROR) not enough room in the output buffer"<<std::endl;
                    break;
                case Z_DATA_ERROR:
                    std::cerr<<"(Z_DATA_ERROR) input data is corrupted or incomplete"<<std::endl;
                    break;
                case Z_STREAM_ERROR:
                    std::cerr<<"(Z_STREAM_ERROR) data stream error"<<std::endl;
                    break;
                }
            }
            convert = (img_type==bot_core::images_t::DISPARITY_ZIPPED);
            break;
        }

        case bot_core::images_t::LEFT:
        case bot_core::images_t::RIGHT:
            // read raw colour data
            colour_data = msg->images[i].data.data();
            // process image according to pixel format
            switch(msg->images[i].pixelformat) {
            case bot_core::image_t::PIXEL_FORMAT_MJPEG:
                // decompress jpeg data directly into previously allocated colour data buffer
                jpeg_decompress((uint8_t*)colour_data, image_size_raw, (uint8_t*)_colorData);
                break;
            default:
                // use as raw data
                _colorData = (ColorType*)colour_data;
            }

            break;
        }
    } // for i over images_t

    std::vector<DepthType> data;

    if(raw_data!=NULL) {
        // found disparity image
        // concatenate 8bit to 16bit using big-endian
        std::vector<uint16_t> data_16bit(_depthData->length());
        memcpy(data_16bit.data(), raw_data, sizeof(uint16_t)*_depthData->length());
        // cast 16bit value to template type 'DepthType'
        std::vector<DepthType> data_typed(data_16bit.begin(), data_16bit.end());
        // disparity to distance
        if(convert) {
            disparity_to_depth(data_typed.data());
        }
#ifdef FILTER_DIST
        for(unsigned int i=0; i<data_typed.size(); i++) {
            data_typed[i] = (data_typed[i]>FILTER_DIST) ? 0 : data_typed[i];
        }
#endif
        data = data_typed;
    }
    else {
        std::cerr<<"no disparity or depth image found"<<std::endl;
    }

    _mutex.lock();

    // cast time from signed to unsigned
    _depthTime = (msg->utime >= 0) ? (uint64_t)msg->utime : 0;
    _colourTime = (msg->utime >= 0) ? (uint64_t)msg->utime : 0;

    // sync
#ifdef CUDA_BUILD
    memcpy(_depthData->hostPtr(), data.data(), sizeof(DepthType)*_depthData->length());
    delete [] raw_data;
    _depthData->syncHostToDevice();
#else
    _depthData = (DepthType*)raw_data;
    // TODO: who is freeing 'raw_data'?
#endif // CUDA_BUILD

    _mutex.unlock();
}

// replace disparity by depth, values are changes inplace (in memory)
template <typename DepthType, typename ColorType>
void LCM_DepthSource<DepthType,ColorType>::disparity_to_depth(DepthType *disparity_img) {
    // Z = (f*b)/d
    // distance_meter = (focal_length_pxl * baseline_meter) / disparity_pxl
    const float factor = _cam_param.focal_length.x * _cam_param.baseline;
    const float sr = _cam_param.subpixel_resolution;

    // compute distance from dispaerity per pixel
    // TODO: Is there a faster way than going thru each pixel individually?
    for(unsigned int i=0; i<_depthData->length(); i++) {
        // deal with disparity 0 (avoid FPE / division by zero)
        disparity_img[i] = (disparity_img[i]!=0.0) ? (factor / (disparity_img[i] * sr)) : 0;
    }
}

} // namespace dart

#endif // LCM_DEPTH_PROVIDER_H
