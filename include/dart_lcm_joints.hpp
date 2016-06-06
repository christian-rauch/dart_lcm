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

    bool setJointNames(const HostOnlyModel *model);

    void initLCM(const std::string topic_name);

    void next();

    void handle_msg_joints(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg);


private:
    lcm::LCM *_lcm;
    std::vector<std::string> _joint_names;
    std::vector<float> _joint_values;
};

} // namespace dart

#endif // DART_LCM_JOINTS_H
