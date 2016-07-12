// LCM
#include <lcm_state_publish.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/robot_state_t.hpp>
#include <lcmtypes/bot_core/joint_state_t.hpp>

#include <sys/time.h>

dart::LCM_StatePublish::LCM_StatePublish(std::string channel, dart::Pose &pose) :
    _channel_prefix(channel), _pose(pose) { }

dart::LCM_StatePublish::~LCM_StatePublish() {}

std::pair< std::vector<std::string>, std::vector<float> > dart::LCM_StatePublish::getJointList() {
    std::vector<std::string> joint_names;
    std::vector<float> joint_values;

    for(unsigned int i=0; i<_pose.getReducedArticulatedDimensions(); i++) {
        const float joint_value = _pose.getReducedArticulation()[i];
        const std::string joint_name = _pose.getReducedName(i);

        // check for fixed joints, e.g. whose value is limited to 0
        if(joint_value==0 && _pose.getReducedMin(i)==_pose.getReducedMax(i))
            continue;

        // add joint name and value if joint is not fixed
        joint_names.push_back(joint_name);
        joint_values.push_back(joint_value);
    }

    return std::make_pair(joint_names, joint_values);
}

bool dart::LCM_StatePublish::publish() {

    if(!getLCM().good())
        return false;

    // list of joint names and their value
    std::pair< std::vector<std::string>, std::vector<float> > joints = getJointList();
    // check for correct amount of joint names and values
    assert(joints.first.size()==joints.second.size());

    // create LCM messages
    bot_core::robot_state_t robot_state_msg;
    bot_core::joint_state_t joint_state_msg;

    // get current time in milliseconds since the epoch
    struct timeval tv;
    gettimeofday(&tv, NULL);
    robot_state_msg.utime = joint_state_msg.utime =\
            (tv.tv_sec * 1000 + tv.tv_usec / 1000);

    // set joint names and values for both messages
    robot_state_msg.num_joints = joint_state_msg.num_joints = joints.first.size();
    robot_state_msg.joint_name = joint_state_msg.joint_name = joints.first;
    robot_state_msg.joint_position = joint_state_msg.joint_position = joints.second;

    // FIXME: We apparently need to set something for each message member to
    // get LCM to encode our message. This should not be necessary.
    robot_state_msg.joint_velocity = joint_state_msg.joint_velocity = \
    robot_state_msg.joint_effort = joint_state_msg.joint_effort =\
            std::vector<float>(joints.second.size());

    // publish messages
    getLCM().publish(_channel_prefix+"_ROBOT", &robot_state_msg);
    getLCM().publish(_channel_prefix+"_JOINTS", &joint_state_msg);

    return true;
}
