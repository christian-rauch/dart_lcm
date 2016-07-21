// LCM
#include <lcm_state_publish.hpp>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/robot_state_t.hpp>
#include <lcmtypes/bot_core/joint_state_t.hpp>

#include <lcm_state_merge.hpp>
#include <lcm_msg_operators.hpp>

#include <sys/time.h>

dart::LCM_StatePublish::LCM_StatePublish(const std::string rep_channel, const std::string pub_channel, dart::Pose &pose, const bool apply_limits) :
    _channel_prefix(pub_channel), _pose(pose)
{
    // subscribe to reported pose for comparison
    // As messages are handled in common thread, all callbacks attached to the
    // same channel should be processed after each other. Hence, the reported
    // robot pose should be the same amongst all instances using this channel.
    if(getLCMSub().good())
        getLCMSub().subscribe(rep_channel, &dart::LCM_StatePublish::store_message, this);
    else
        throw std::runtime_error("LCM is not good. Not subscribing.");

    if(apply_limits)
        getLimits(pose);
}

dart::LCM_StatePublish::~LCM_StatePublish() { }

std::pair< std::vector<std::string>, std::vector<float> > dart::LCM_StatePublish::getJointList() {
    std::vector<std::string> joint_names;
    std::vector<float> joint_values;

    for(int i=0; i<_pose.getReducedArticulatedDimensions(); i++) {
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

void dart::LCM_StatePublish::getLimits(const dart::Pose &pose) {
    for(int i=0; i<pose.getReducedArticulatedDimensions(); i++)
        _limits[pose.getReducedName(i)] = std::make_pair(pose.getReducedMin(i), pose.getReducedMax(i));
}

bool dart::LCM_StatePublish::publish() {

    if(!getLCMSub().good())
        return false;

    // list of joint names and their value
    std::pair< std::vector<std::string>, std::vector<float> > joints = getJointList();
    // check for correct amount of joint names and values
    assert(joints.first.size()==joints.second.size());

    // create LCM messages
    bot_core::joint_state_t est_joint_state;

    // get current time in milliseconds since the epoch
    struct timeval tv;
    gettimeofday(&tv, NULL);
    est_joint_state.utime = (tv.tv_sec * 1000 + tv.tv_usec / 1000);

    // set joint names and values for both messages
    est_joint_state.num_joints = joints.first.size();
    est_joint_state.joint_name = joints.first;
    est_joint_state.joint_position = joints.second;

    // FIXME: We apparently need to set something for each message member to
    // get LCM to encode our message. This should not be necessary.
    est_joint_state.joint_velocity = est_joint_state.joint_effort = std::vector<float>(joints.second.size());

    // publish messages
    getLCMPub().publish(_channel_prefix+"ESTIMATE_JOINTS", &est_joint_state);

    if(_reported.joint_name.size()!=0) {
        bot_core::robot_state_t robot_state_msg, robot_state_diff_msg;
        _mutex.lock();
        std::tie(robot_state_msg, robot_state_diff_msg) = LCM_StateMerge::merge(_reported, est_joint_state, _limits);
        _mutex.unlock();
        getLCMPub().publish(_channel_prefix+"REPORTED", &_reported);
        getLCMPub().publish(_channel_prefix+"ESTIMATE_STATE", &robot_state_msg);
        getLCMPub().publish(_channel_prefix+"ESTIMATE_DIFF", &robot_state_diff_msg);
    }

    return true;
}
