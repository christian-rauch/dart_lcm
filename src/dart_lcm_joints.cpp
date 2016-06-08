#include "dart_lcm_joints.hpp"
#include <limits>
#include <algorithm>

namespace dart {

LCM_JointsProvider::LCM_JointsProvider() {
    _lcm = new lcm::LCM();
}

LCM_JointsProvider::~LCM_JointsProvider() {
    if(_lcm->good())
        delete _lcm;
}

bool LCM_JointsProvider::setJointNames(const std::vector<std::string> &joint_names) {
    _joint_names = joint_names;
    return true;
}

bool LCM_JointsProvider::setJointNames(const HostOnlyModel &model) {
    _joint_names.resize(model.getNumJoints());
    for(unsigned int j=0; j<_joint_names.size(); j++) {
        _joint_names[j] = model.getJointName(j);
    }
}

void LCM_JointsProvider::initLCM(const std::string topic_name) {
    if(_lcm->good()) {
        _lcm->subscribe(topic_name, &LCM_JointsProvider::handle_msg_joints, this);
    }
}

int LCM_JointsProvider::next(const int time_ms) {
    int ret = -1;
    // check if joint names are known
    if(_joint_names.empty()) {
        std::cerr<<"no joints specified, you should add joint names directly or by model"<<std::endl;
    }

    // wait for next message (for defined time)
    if(_lcm->good() && !_joint_names.empty()) {
        ret = (time_ms>0) ? _lcm->handleTimeout(time_ms) : _lcm->handle();
    }

    // common return codes for blocking and timeour handles
    if(ret>=0 && time_ms>0) {
        ret = (ret == 0) ? -2 : 0;
    }

    return ret;
}

void LCM_JointsProvider::handle_msg_joints(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg) {
    // allocate memory for amount of expected joints
    _joint_values.resize(_joint_names.size());

    // hashtable for faster search of joint names and values
    std::map<std::string, float> joints;
    std::transform(msg->joint_name.begin(), msg->joint_name.end(),  // key range
                   msg->joint_position.begin(),                     // value range
                   std::inserter(joints, joints.end()),             // target map
                   std::make_pair<std::string const&, float const&>); // key-value pair

    // order '_joint_values' as they appear in '_joint_names'
    // We need to check for the existence of a key before using the [] operator,
    // otherwise a new entry with this key and default constructed element will
    // be generated. Use NaN value if key is not present to determine omitted
    // joints later using std::isnan().
    for(unsigned int i=0; i<_joint_names.size(); i++) {
        _joint_values[i] = (joints.count(_joint_names[i])!=0) ? joints[_joint_names[i]] : NAN;
    }
}

} // namespace dart
