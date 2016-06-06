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

bool LCM_JointsProvider::setJointNames(const HostOnlyModel *model) {
    _joint_names.resize(model->getNumJoints());
    for(unsigned int j=0; j<_joint_names.size(); j++) {
        _joint_names[j] = model->getJointName(j);
    }
}

void LCM_JointsProvider::initLCM(const std::string topic_name) {
    if(_lcm->good()) {
        _lcm->subscribe(topic_name, &LCM_JointsProvider::handle_msg_joints, this);
    }
}

void LCM_JointsProvider::next() {
    if(_joint_names.empty()) {
        std::cerr<<"no joints specified, you should add joint names directly or by model"<<std::endl;
    }

    if(_lcm->good() && !_joint_names.empty()) {
        _lcm->handle();
    }
}

void LCM_JointsProvider::handle_msg_joints(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg) {
    //
    _joint_values.clear();
}

}
