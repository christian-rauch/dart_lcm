#include "dart_lcm_joints.hpp"

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

    // check received joint values
    if(_joint_names.size() != _joint_values.size()) {
        std::cerr<<"received different amount of joint values ("<<_joint_values.size()<<") than defined by model ("<<_joint_names.size()<<")"<<std::endl;
    }

    // common return codes for blocking and timeour handles
    if(ret>=0 && time_ms>0) {
        ret = (ret == 0) ? -2 : 0;
    }

    return ret;
}

void LCM_JointsProvider::handle_msg_joints(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg) {
    //
    _joint_values.clear();
}

}
