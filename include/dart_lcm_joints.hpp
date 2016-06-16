#ifndef DART_LCM_JOINTS_H
#define DART_LCM_JOINTS_H

// LCM
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/robot_state_t.hpp>

// DART
#include <dart/model/host_only_model.h>

// threading
#include <thread>
#include <atomic>
#include <mutex>
#include <boost/thread.hpp>

namespace  dart {

class LCM_JointsProvider
{

public:
    /**
     * @brief LCM_JointsProvider default constructor
     */
    LCM_JointsProvider();

    ~LCM_JointsProvider();

    /**
     * @brief setJointNames initialize LCM with expected order of joint names
     * @param joint_names list of joint names
     */
    void setJointNames(const std::vector<std::string> &joint_names);

    /**
     * @brief setJointNames initialize LCM with expected order of joint names
     * @param model DART model with initialized frames
     */
    void setJointNames(const HostOnlyModel &model);

    void initLCM(const std::string topic_name, const bool threading = false);

    /**
     * @brief next wait for next messages
     * @param time_ms optional timeout in milliseconds
     * @return 0 on success
     * @return -1 on failure
     * @return -2 on timeout
     */
    int next(const int time_ms = 0);

    void handle_msg_joints(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const bot_core::robot_state_t* msg);

    std::vector<float> getJointValues() {
        std::lock_guard<boost::shared_mutex> lck(_mutex);
        return _joint_values;
    }

    dart::SE3 getTransformWorldToRobot(){
        std::lock_guard<boost::shared_mutex> lck(_mutex);
        return _T_wr;
    }

    std::map<std::string, float> getJointsNameValue() {
        std::lock_guard<boost::shared_mutex> lck(_mutex);
        return _joints_name_value;
    }

private:
    lcm::LCM _lcm;
    std::vector<std::string> _joint_names;
    std::vector<float> _joint_values;
    std::map<std::string, float> _joints_name_value;
    dart::SE3 _T_wr;    // transformation world to robot

    /**
     * @brief _handle_thread thread object that handles LCM messages, e.g. waits for incomming messages
     */
    std::thread _handle_thread;

    /**
     * @brief _thread_running atomic flag to check if a thread is already running
     */
    std::atomic<bool> _thread_running;

    /**
     * @brief _mutex shared mutex to lock access to joint values.
     *
     * Use this whenever accessing variables in parallel. Use "std::lock_guard<boost::shared_mutex> lck(_mutex)"
     * for getter methods before a critical variables is returned to ensure the _mutex is locked and
     * will automatically be unlocked when "lck" is destroyed (e.g. gets out of scope after function return).
     */
    boost::shared_mutex _mutex;

};

} // namespace dart

#endif // DART_LCM_JOINTS_H
