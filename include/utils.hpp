#ifndef UTILS_HPP
#define UTILS_HPP

#include <vector>
#include <string>
#include <map>

namespace dart {

template<typename T = float>
std::map<std::string, T> vector_to_map(
        const std::vector<std::string> &names,
        const std::vector<T> &values)
{
    // test vector size
    assert(names.size()==values.size());

    std::map<std::string, T> joint_map;

    for(unsigned int i=0; i<names.size(); i++)
        joint_map[names[i]] = values[i];

    return joint_map;
}

} // namespace dart

#endif // UTILS_HPP
