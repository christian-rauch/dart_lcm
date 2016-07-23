#ifndef LCM_MSG_OPERATORS_HPP
#define LCM_MSG_OPERATORS_HPP

#include <lcmtypes/bot_core.hpp>

static bot_core::vector_3d_t operator-(const bot_core::vector_3d_t &lhs, const bot_core::vector_3d_t &rhs) {
    bot_core::vector_3d_t res;
    res.x = lhs.x - rhs.x;
    res.y = lhs.y - rhs.y;
    res.z = lhs.z - rhs.z;
    return res;
}

static bot_core::quaternion_t operator-(const bot_core::quaternion_t &lhs, const bot_core::quaternion_t &rhs) {
    bot_core::quaternion_t res;
    res.x = lhs.x - rhs.x;
    res.y = lhs.y - rhs.y;
    res.z = lhs.z - rhs.z;
    res.w = lhs.w - rhs.w;
    return res;
}

static bot_core::position_3d_t operator-(const bot_core::position_3d_t &lhs, const bot_core::position_3d_t &rhs) {
    bot_core::position_3d_t res;
    res.translation = lhs.translation - rhs.translation;
    res.rotation = lhs.rotation - rhs.rotation;
    return res;
}

static bot_core::twist_t operator-(const bot_core::twist_t &lhs, const bot_core::twist_t &rhs) {
    bot_core::twist_t res;
    res.linear_velocity = lhs.linear_velocity - rhs.linear_velocity;
    res.angular_velocity = lhs.angular_velocity - rhs.angular_velocity;
    return res;
}

template<typename T>
static std::vector<T> vec_diff(const std::vector<T> &v1, const std::vector<T> &v2) {
    assert(v1.size()==v2.size());

    std::vector<T> res(v1.size());
    for(unsigned int i=0; i<res.size(); i++)
        res[i] = v1[i] - v2[i];

    return res;
}

static bot_core::force_torque_t operator-(const bot_core::force_torque_t &lhs, const bot_core::force_torque_t &rhs) {
    bot_core::force_torque_t res;
    res.l_foot_force_z = lhs.l_foot_force_z - rhs.l_foot_force_z;
    res.l_foot_torque_x = lhs.l_foot_torque_x - rhs.l_foot_torque_x;
    res.l_foot_torque_y = lhs.l_foot_torque_y - rhs.l_foot_torque_y;
    res.r_foot_force_z = lhs.r_foot_force_z - rhs.r_foot_force_z;
    res.r_foot_torque_x = lhs.r_foot_torque_x - rhs.r_foot_torque_x;
    res.r_foot_torque_y = lhs.r_foot_torque_y - rhs.r_foot_torque_y;

    for(unsigned int i=0; i<3; i++) {
        res.l_hand_force[i] = lhs.l_hand_force[i] - rhs.l_hand_force[i];
        res.l_hand_torque[i] = lhs.l_hand_torque[i] - rhs.l_hand_torque[i];
        res.r_hand_force[i] = lhs.r_hand_force[i] - rhs.r_hand_force[i];
        res.r_hand_torque[i] = lhs.r_hand_torque[i] - rhs.r_hand_torque[i];
    }

    return res;
}

static bot_core::robot_state_t operator-(const bot_core::robot_state_t &lhs, const bot_core::robot_state_t &rhs) {
    assert(lhs.num_joints==rhs.num_joints);
    assert(lhs.joint_name==rhs.joint_name);

    bot_core::robot_state_t res;

    // use left hand side as reference for values that are not subtracted
    res.utime = lhs.utime;
    res.num_joints = lhs.num_joints;
    res.joint_name = lhs.joint_name;

    res.pose = lhs.pose - rhs.pose;
    res.twist = lhs.twist - rhs.twist;
    res.joint_position = vec_diff(lhs.joint_position, rhs.joint_position);
    res.joint_velocity = vec_diff(lhs.joint_velocity, rhs.joint_velocity);
    res.joint_effort = vec_diff(lhs.joint_effort, rhs.joint_effort);
    res.force_torque = lhs.force_torque - rhs.force_torque;

    return res;
}


#endif // LCM_MSG_OPERATORS_HPP
