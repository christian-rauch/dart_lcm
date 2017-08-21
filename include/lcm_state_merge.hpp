#ifndef LCM_STATEMERGE_H
#define LCM_STATEMERGE_H

#include <vector>
#include <string>
#include <tuple>

#include "utils.hpp"

namespace LCM_StateMerge {

typedef std::map<std::string, std::pair<float, float>> LimitMap;

/**
 * @brief apply_limits apply joint limits to a LCM message containing joint_position
 * @param ref message on which to enforce limits
 * @param limits joint names and their lower and upper limit
 */
template<typename MSG_R>
void apply_limits(MSG_R &ref, const LimitMap &limits) {
    for(unsigned int i = 0; i<uint(ref.num_joints); i++) {
        const std::string jname = ref.joint_name[i];
        if(limits.count(jname)) {
            ref.joint_position[i] = std::max(ref.joint_position[i], limits.at(jname).first);
            ref.joint_position[i] = std::min(ref.joint_position[i], limits.at(jname).second);
        }
    }
}

/**
 * @brief merge_raw merge reference and estimated joint values by replacing reference values by estimated values
 * @param ref_names vector of names for reference joints
 * @param ref_values vector of values for reference joints
 * @param est_names vector of names for estimated joints
 * @param est_values vector of values for estimated joints
 * @return tuple with name of reference joints in first place, merged values in second place
 */
template<typename T = float>
std::tuple<std::vector<std::string>, std::vector<T>>
merge_raw(const std::vector<std::string> &ref_names,
          const std::vector<T> &ref_values,
          const std::vector<std::string> &est_names,
          const std::vector<T> &est_values)
{
    const std::map<std::string, T> est_map = dart::vector_to_map(est_names, est_values);

    std::vector<T> merge = ref_values;
    for(unsigned int i=0; i<ref_names.size(); i++) {
        const std::string jname = ref_names[i];
        if(est_map.count(jname)) {
            merge[i] = est_map.at(jname);
        }
    }

    return std::make_tuple(ref_names, merge);
}

/**
 * @brief merge
 * @param ref reference robot state
 * @param est estimated robot state (or joint values)
 * @param limits joint limits to be enforced on reported state (optional)
 * @return tuple of merged robot state and its difference to the reference
 */
template<typename MSG_R, typename MSG_E>
std::tuple<MSG_R, MSG_R> merge(MSG_R &ref, const MSG_E &est,
                               const LimitMap &limits = LimitMap())
{
    if(limits.size()!=0)
        apply_limits(ref, limits);

    const auto merged = merge_raw(ref.joint_name, ref.joint_position, est.joint_name, est.joint_position);

    MSG_R state_merged = ref;
    state_merged.joint_position = std::get<1>(merged);

    MSG_R state_diff = ref - state_merged;

    return std::make_tuple(state_merged, state_diff);
}

} // namespace LCM_StateMerge

#endif // LCM_STATEMERGE_H
