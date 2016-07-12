#ifndef LCM_STATEPUBLISH_HPP
#define LCM_STATEPUBLISH_HPP

#include "lcm_provider_base.hpp"
#include <dart/pose/pose.h>

namespace dart {

class LCM_StatePublish : public LCM_CommonBase {
private:
    std::string _channel_prefix;    //!< prefix that is prepended to channel name
    const dart::Pose &_pose;        //!< reference to estimated model pose

    /**
     * @brief getJointList generate list of joint names and values from dart pose
     * @return pair of joint name list and joint value list
     */
    std::pair< std::vector<std::string>, std::vector<float> > getJointList();

public:
    /**
     * @brief LCM_StatePublish construct LCM publisher
     * @param channel channel name prefix to publish on
     * @param pose estimated DART pose of model whose values are continuously updated
     */
    LCM_StatePublish(std::string channel, dart::Pose &pose);

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
