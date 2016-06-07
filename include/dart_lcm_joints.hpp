#ifndef DART_LCM_JOINTS_H
#define DART_LCM_JOINTS_H

#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/robot_state_t.hpp>
#include <dart/model/host_only_model.h>

namespace  dart {

class LCM_JointsProvider
{

public:
    LCM_JointsProvider();
    ~LCM_JointsProvider();

    bool setJointNames(const std::vector<std::string> &joint_names);

    bool setJointNames(const HostOnlyModel &model);

    void initLCM(const std::string topic_name);

    /**
     * @brief next wait for next messages
     * @param time_ms optional timeout in milliseconds
     * @return 0 on success
     * @return -1 on failure
     * @return -2 on timeout
     */
    int next(const int time_ms = 0);

    void handle_msg_joints(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg);

    const std::vector<float> &getJointValues() const {
        return _joint_values;
    }


private:
    lcm::LCM *_lcm;
    std::vector<std::string> _joint_names;
    std::vector<float> _joint_values;
};

} // namespace dart

#endif // DART_LCM_JOINTS_H
