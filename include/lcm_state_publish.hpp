#ifndef LCM_STATEPUBLISH_HPP
#define LCM_STATEPUBLISH_HPP

#include "lcm_provider_base.hpp"
#include <dart/pose/pose.h>
#include <lcmtypes/bot_core/robot_state_t.hpp>

#include <mutex>

namespace dart {

class LCM_StatePublish : public LCM_CommonBase {
private:
    std::string _channel_prefix;    //!< prefix that is prepended to channel name
    const dart::Pose &_pose;        //!< reference to estimated model pose
    bot_core::robot_state_t _reported; //!< last received pose
    std::mutex _mutex;              //!< mutex to synchronize access to _reported pose
    std::map<std::string, std::pair<float, float>> _limits; //!< joint limits

    /**
     * @brief getJointList generate list of joint names and values from dart pose
     * @return pair of joint name list and joint value list
     */
    std::pair< std::vector<std::string>, std::vector<float> > getJointList();

    /**
     * @brief getLimits fetch joint limits from DART pose
     * @param pose DART pose with limits on joints
     */
    void getLimits(const dart::Pose &pose);

    void store_message(const lcm::ReceiveBuffer *rbuf, const std::string &channel, const bot_core::robot_state_t *msg) {
        if(_mutex.try_lock()) {
            // copy reported pose
            _reported = *msg;
            _mutex.unlock();
        }
    }

public:
    /**
     * @brief LCM_StatePublish construct LCM publisher
     * @param rep_channel channel to listen for reported pose
     * @param pub_channel channel name prefix to publish on
     * @param pose estimated DART pose of model whose values are continuously updated
     * @param apply_limits flag to read and enforce joint limits from DART pose
     */
    LCM_StatePublish(const std::string rep_channel, const std::string pub_channel, dart::Pose &pose, const bool apply_limits = true);

    ~LCM_StatePublish();

    /**
     * @brief publish_estimate read current joint values from pose and publish messages
     * @return true on success
     * @return false on failure
     */
    bool publish_estimate();
};

}

#endif // LCM_STATEPUBLISH_HPP
