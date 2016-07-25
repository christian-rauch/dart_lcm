#include "lcm_frame_pose_publish.hpp"
#include <lcm_msg_operators.hpp>

#include <cmath>

dart::LCM_FramePosePublish::LCM_FramePosePublish(const std::string &channel_pref, dart::Model &rep, dart::MirroredModel &est) : _channel(channel_pref), _rep(rep), _est(est){ }

bot_core::position_3d_t dart::LCM_FramePosePublish::MSGfromSE3(const dart::SE3 &transform) {
    const dart::se3 pose = dart::se3FromSE3(transform);
    bot_core::position_3d_t msg;
    msg.translation.x = pose.p[0];
    msg.translation.y = pose.p[1];
    msg.translation.z = pose.p[2];
    const float3 rot = dart::eulerFromSE3(transform);
    // We are using the rotation field (quaternion_t) the wrong way to prevent the
    // forth and backwards conversion of Euler to Quaternion. The 'w' is set to
    // NAN to indicate that this is not a valid Quaternion.
    msg.rotation.x = rot.x; // phi
    msg.rotation.y = rot.y; // theta
    msg.rotation.z = rot.z; // psi
    msg.rotation.w = NAN;
    return msg;
}

bool dart::LCM_FramePosePublish::publish_frame_pose(const std::string &frame) {
//    const dart::SE3 rep_frame_pose = _rep.getTransformCameraToFrame(_rep.getJointIdByName(frame));
//    const dart::SE3 est_frame_pose = _est.getTransformCameraToFrame(_est.getJointIdByName(frame));

//    const dart::SE3 rep_frame_pose = _rep.getTransformFrameToCamera(_rep.getJointIdByName(frame));
//    const dart::SE3 est_frame_pose = _est.getTransformFrameToCamera(_est.getJointIdByName(frame));

//    const dart::SE3 rep_frame_pose = _rep.getTransformModelToFrame(_rep.getJointIdByName(frame));
//    const dart::SE3 est_frame_pose = _est.getTransformModelToFrame(_est.getJointIdByName(frame));

    const dart::SE3 rep_frame_pose = _rep.getTransformFrameToModel(_rep.getJointIdByName(frame));
    const dart::SE3 est_frame_pose = _est.getTransformFrameToModel(_est.getJointIdByName(frame));

    const bot_core::position_3d_t msg_rep = MSGfromSE3(rep_frame_pose);
    const bot_core::position_3d_t msg_est = MSGfromSE3(est_frame_pose);
    bot_core::position_3d_t diff = msg_rep - msg_est;
    // wrap angular error around -PI and PI to prevent errors > PI
    diff.rotation.x = M_PI - std::abs(std::abs(diff.rotation.x) - M_PI);
    diff.rotation.y = M_PI - std::abs(std::abs(diff.rotation.y) - M_PI);
    diff.rotation.z = M_PI - std::abs(std::abs(diff.rotation.z) - M_PI);

    publish(_channel+"_POSE_"+frame+"_REP", &msg_rep);
    publish(_channel+"_POSE_"+frame+"_EST", &msg_est);
    publish(_channel+"_POSE_"+frame+"_DIFF", &diff);

    return true;
}
