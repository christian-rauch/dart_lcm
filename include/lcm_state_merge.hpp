#ifndef LCM_STATEMERGE_H
#define LCM_STATEMERGE_H

#include <vector>
#include <string>
#include <tuple>

#include "utils.hpp"

namespace LCM_StateMerge {

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
 * @param est estimated robot state
 * @return tuple of merged robot state and its difference to the reference
 */
template<typename MSG>
std::tuple<MSG, MSG> merge(const MSG &ref, const MSG &est) {
    const auto merged = merge_raw(ref.joint_name, ref.joint_position, est.joint_name, est.joint_position);

    MSG state_merged = ref;
    state_merged.joint_position = std::get<1>(merged);

    MSG state_diff = ref - state_merged;

    return std::make_tuple(state_merged, state_diff);
}

} // namespace LCM_StateMerge

#endif // LCM_STATEMERGE_H
