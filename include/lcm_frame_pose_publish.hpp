#ifndef LCM_FRAMEPOSEPUBLISH_HPP
#define LCM_FRAMEPOSEPUBLISH_HPP

#include "lcm_provider_base.hpp"
#include <dart/model/mirrored_model.h>
#include <lcmtypes/bot_core/position_3d_t.hpp>

namespace dart {

class LCM_FramePosePublish : public LCM_CommonBase {
public:
    LCM_FramePosePublish(const std::string &channel_pref, dart::Model &rep, dart::MirroredModel &est);

    bool publish_frame_pose(const std::string &frame);

    static bot_core::position_3d_t MSGfromSE3(const dart::SE3 &transform);

private:
    const std::string _channel;
    dart::Model &_rep;
    dart::MirroredModel &_est;
};

} // namespace dart

#endif // LCM_FRAMEPOSEPUBLISH_HPP
