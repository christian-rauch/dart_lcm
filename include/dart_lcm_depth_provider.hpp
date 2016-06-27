#ifndef LCM_DEPTH_PROVIDER_H
#define LCM_DEPTH_PROVIDER_H

// DART header
#include <dart/depth_sources/depth_source.h>
#include <dart/util/mirrored_memory.h>

// LCM and message header
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include "lcm_singelton.hpp"

// threading
#include <thread>
#include <mutex>
#include <atomic>
#include <boost/thread.hpp>

// stream input/output
#include <iostream>

// zlib de/-compression
#include <zlib.h>

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
    float subpixel_resolution;
};

////////////////////////////////////////////////////////////////////////////////
/// LCM_DepthSource class definition
///
template <typename DepthType, typename ColorType>
class LCM_DepthSource : public dart::DepthSource<DepthType,ColorType> {
private:
    /**
     * @brief lcm pointer to LCM
     */
    lcm::LCM lcm;

#ifdef CUDA_BUILD
    /**
     * @brief _depthData pointer to class for managing memory representation at host system and GPU memory
     */
    dart::MirroredVector<DepthType> * _depthData;
#else
    /**
     * @brief _depthData pointer to regular array storing depth values
     */
    DepthType * _depthData;
#endif // CUDA_BUILD

    /**
     * @brief _depthTime timestamp (seconds since the epoch) of depth data
     */
    uint64_t _depthTime;

    /**
     * @brief _cam_param internal structure to store camera parameters
     */
    StereoCameraParameter _cam_param;

    /**
     * @brief _ScaleToMeters optional scale that is multiplied with internally stored values to obtain measurement in meter. It is applied just before the backprojection.
     */
    float _ScaleToMeters;

    /**
     * @brief _timeout_ms optional timeout in milliseconds when waiting for messages
     */
    int _timeout_ms;

    /**
     * @brief _handle_thread thread object that handles LCM messages, e.g. waits for incomming messages
     */
    std::thread _handle_thread;

    /**
     * @brief _thread_running atomic flag to check if a threasd is already running
     */
    std::atomic<bool> _thread_running;

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

    /**
     * @brief initLCM initializing LCM subscriber to images_t topic
     *
     * This method sets up the subscription for the requested channel. By default, the handling of messages (and thus advance()) will block. Set threading to true to wait for incomming messages in a dedicated thread. Alternatively to wait for incomming messages in a single thread, a timeout value van be set. This value is not used if threading is true.
     *
     * @param channel topic name of stereo camera, e.g. "CAMERA"
     * @param threading wait for incomming messages in dedicated thread
     * @param timeout_ms optional timeout in milliseconds when waiting for messages
     * @return true on success, false otherwise
     */
    bool initLCM(const std::string &img_channel, const bool threading = false, const int timeout_ms = 0);

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

    lcm = LCMSingelton::getLCM();

    // thread not running at initilization
    _thread_running = false;

    // depth source properties
    this->_isLive = true; // no way to control LCM playback from here
    this->_hasColor = false; // only depth for now
    this->_colorWidth = 0;
    this->_colorHeight = 0;
    this->_hasTimestamps = true;
    this->_frame = 0; // initialize first frame

    // camera properties
    _cam_param = param;
    this->_focalLength = param.focal_length;
    this->_principalPoint = param.camera_center;
    this->_depthWidth = param.width;
    this->_depthHeight = param.height;

    _ScaleToMeters = scale;

    // allocate memory for depth image
#ifdef CUDA_BUILD
    _depthData = new dart::MirroredVector<DepthType>(this->_depthWidth*this->_depthHeight);
#else
    _depthData = new DepthType[this->_depthWidth*this->_depthHeight];
#endif // CUDA_BUILD
}

template <typename DepthType, typename ColorType>
LCM_DepthSource<DepthType,ColorType>::~LCM_DepthSource() {
#ifdef CUDA_BUILD
    delete _depthData;
#else
    delete [] _depthData;
#endif // CUDA_BUILD
}

template <typename DepthType, typename ColorType>
void LCM_DepthSource<DepthType,ColorType>::setFrame(const uint frame) {
    // nothing to do, we cannot control LCM playbag from here
    if(this->_isLive) return;
}

template <typename DepthType, typename ColorType>
void LCM_DepthSource<DepthType,ColorType>::advance() {
    if(!_thread_running) {
        // wait (block) for new messages
        (_timeout_ms>0) ? lcm.handleTimeout(_timeout_ms) : lcm.handle();
    }
    // ignore call if messages are handled in thread

    // TODO: enable reading from log file directly
}

template <typename DepthType, typename ColorType>
bool LCM_DepthSource<DepthType,ColorType>::hasRadialDistortionParams() const {
    return false;
}

template <typename DepthType, typename ColorType>
bool LCM_DepthSource<DepthType,ColorType>::initLCM(const std::string &img_channel, const bool threading, const int timeout_ms) {
    if(!lcm.good()) {
        return false;
    }

    lcm.subscribe(img_channel, &LCM_DepthSource<DepthType, ColorType>::imgHandle, this);

    _timeout_ms = timeout_ms;

    if(threading) {
        if(!_thread_running) {
            // create new thread using lambda function for looping
            _thread_running = true;
            _handle_thread = std::thread([this]{
                while(lcm.good()) lcm.handle();
                _thread_running = false;
            });
            _handle_thread.detach();
        }
        else {
            std::cerr<<"You try to initialize a thread more than once. Ignoring this request until original thread is finished."<<std::endl;
        }
    }
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
            raw_data = msg->images[i].data.data();
            break;

        // zlib compressed disparity image data
        case bot_core::images_t::DISPARITY_ZIPPED:
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
        disparity_to_depth(data_typed.data());
        data = data_typed;
    }
    else {
        std::cerr<<"no disparity image found"<<std::endl;
    }

    _mutex.lock();

    // cast time from signed to unsigned
    _depthTime = (msg->utime >= 0) ? (uint64_t)msg->utime : 0;

    // sync
#ifdef CUDA_BUILD
    memcpy(_depthData->hostPtr(), data.data(), sizeof(DepthType)*_depthData->length());
    delete [] raw_data;
    _depthData->syncHostToDevice();
#else
    _depthData = raw_data;
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
