#include "lcm_frame_pose_publish.hpp"
#include <lcm_msg_operators.hpp>



dart::LCM_FramePosePublish::LCM_FramePosePublish(const std::string &channel_pref, dart::Model &rep, dart::MirroredModel &est) : _channel(channel_pref), _rep(rep), _est(est){ }

bot_core::position_3d_t dart::LCM_FramePosePublish::MSGfromSE3(const dart::SE3 &transform) {
    bot_core::position_3d_t msg;
    msg.translation.x = transform.r0.w;
    msg.translation.y = transform.r1.w;
    msg.translation.z = transform.r2.w;
    const float3 rot = dart::eulerFromSE3(transform);
    // We are using the rotation field (quaternion_t) the wrong way to prevent the
    // forth and backwards conversion of Euler to Quaternion. The 'w' is set to
    // NAN to indicate that this is not a valid Quaternion.
    msg.rotation.x = rot.x;
    msg.rotation.y = rot.y;
    msg.rotation.z = rot.z;
    msg.rotation.w = NAN;
    return msg;
}

bool dart::LCM_FramePosePublish::publish_frame_pose(const std::string &frame) {
    const dart::SE3 rep_frame_pose = _rep.getTransformCameraToFrame(_rep.getJointIdByName(frame));
    const dart::SE3 est_frame_pose = _est.getTransformCameraToFrame(_est.getJointIdByName(frame));

    const bot_core::position_3d_t msg_rep = MSGfromSE3(rep_frame_pose);
    const bot_core::position_3d_t msg_est = MSGfromSE3(est_frame_pose);
    const bot_core::position_3d_t diff = msg_rep - msg_est;

    publish(_channel+"_POSE_"+frame+"_REP", &msg_rep);
    publish(_channel+"_POSE_"+frame+"_EST", &msg_est);
    publish(_channel+"_POSE_"+frame+"_DIFF", &diff);

    return true;
}
