#ifndef LCM_STATEPUBLISH_HPP
#define LCM_STATEPUBLISH_HPP

#include "lcm_provider_base.hpp"
#include <dart/pose/pose.h>
#include <lcmtypes/bot_core/robot_state_t.hpp>

namespace dart {

class LCM_StatePublish : public LCM_CommonBase {
private:
    std::string _channel_prefix;    //!< prefix that is prepended to channel name
    const dart::Pose &_pose;        //!< reference to estimated model pose
    bot_core::robot_state_t _reported; //!< last received pose

    /**
     * @brief getJointList generate list of joint names and values from dart pose
     * @return pair of joint name list and joint value list
     */
    std::pair< std::vector<std::string>, std::vector<float> > getJointList();

    void store_message(const lcm::ReceiveBuffer *rbuf, const std::string &channel, const bot_core::robot_state_t *msg) {
        // copy reported pose
        _reported = *msg;
    }

public:
    /**
     * @brief LCM_StatePublish construct LCM publisher
     * @param rep_channel channel to listen for reported pose
     * @param pub_channel channel name prefix to publish on
     * @param pose estimated DART pose of model whose values are continuously updated
     */
    LCM_StatePublish(const std::string rep_channel, const std::string pub_channel, dart::Pose &pose);

    ~LCM_StatePublish();

    /**
     * @brief publish read current joint values from pose and publish messages
     * @return true on success
     * @return false on failure
     */
    bool publish();
};

}

#endif // LCM_STATEPUBLISH_HPP
