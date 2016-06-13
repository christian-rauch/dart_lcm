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
    std::transform(msg->joint_name.begin(), msg->joint_name.end(),  // key range
                   msg->joint_position.begin(),                     // value range
                   std::inserter(_joints_name_value, _joints_name_value.end()), // target map
                   std::make_pair<std::string const&, float const&>); // key-value pair

    // order '_joint_values' as they appear in '_joint_names'
    // We need to check for the existence of a key before using the [] operator,
    // otherwise a new entry with this key and default constructed element will
    // be generated. Use NaN value if key is not present to determine omitted
    // joints later using std::isnan().
    for(unsigned int i=0; i<_joint_names.size(); i++) {
        const std::string cur_joint = _joint_names[i];
        // if joint is not available, set value in the joint map to NAN
        if(_joints_name_value.count(cur_joint)==0) {
            _joints_name_value[cur_joint] = NAN;
        }
        _joint_values[i] = _joints_name_value.at(cur_joint);
    }

    // get robot pose, transformation from world frame  to robot root frame
    const bot_core::vector_3d_t t = msg->pose.translation;
    const bot_core::quaternion_t q = msg->pose.rotation;
    // quaternion to euler angles
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_from_Quaternion
    float phi, theta, psi;
    phi = atan2(2*(q.w*q.x + q.y*q.z), 1-2*(pow(q.x,2) + pow(q.y,2)));
    theta = asin(2*(q.w*q.y - q.z*q.x));
    psi = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(pow(q.y,2) + pow(q.z,2)));

    _T_wr = dart::SE3Fromse3(dart::se3(t.x, t.y, t.z, phi, theta, psi));
}

} // namespace dart
