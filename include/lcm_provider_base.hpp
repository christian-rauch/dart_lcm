#ifndef LCM_COMMON_BASE_HPP
#define LCM_COMMON_BASE_HPP

#include <lcm/lcm-cpp.hpp>

#include <thread>

#include <iostream>

/**
 * @brief The LCM_CommonBase class
 *
 * Base class for instantiating LCM and handling messages in thread. It provides
 * a common LCM and thread object such that message handling is only done once
 * in a process that uses multiple inherited instances of LCM_CommonBase. Hence,
 * all LCM related classes in DART should inherit from this base class and add
 * their specific subscriber and publisher using the getLCM() method. The common
 * LCM parameters given to the constructor can be provided by init() before any
 * inherited class is instanciated.
 */
class LCM_CommonBase {
public:
    /**
     * @brief setProvider initialize provider parameters for LCM before it is instantiated
     * @param param LCM provider string
     */
    static void setProvider(const std::string &param) {
        if(_lcm_sub == NULL) {
            _param = param;
        }
        else {
            std::cerr<<"LCM object is already instantiated. Calling setProvider() will have no effect."<<std::endl;
        }
    }

    /**
     * @brief good forwarding of lcm::LCM::good()
     * @return
     */
    static bool good() {
        return (_lcm_sub->good() && _lcm_pub->good());
    }

    /**
     * @brief publish forwarding of lcm::LCM::publish(...)
     * @param args parameters for publish(...)
     * @return
     */
    template<typename... Args>
    static int publish(Args&&... args) {
        return _lcm_pub->publish(std::forward<Args>(args)...);
    }

    /**
     * @brief subscribe forwarding of lcm::LCM::subscribe(...)
     * @param args parameters for subscribe(...)
     * @return
     */
    template<typename... Args>
    static lcm::Subscription* subscribe(Args&&... args) {
        return _lcm_sub->subscribe(std::forward<Args>(args)...);
    }

protected:
    LCM_CommonBase() {
        start();
    }

    ~LCM_CommonBase() { delete _lcm_sub; delete _lcm_pub;}

private:
    static lcm::LCM *_lcm_sub;  /// LCM object pointer for subscribing
    static lcm::LCM *_lcm_pub;  /// LCM object pointer for publishing
    static std::string _param;  /// LCM provider string
    static std::thread _thread; /// thread

    /**
     * @brief start instantiate LCM and run handle thread
     */
    void start() {
        // create and configure LCM subscription object
        if(_lcm_sub == NULL) {
            _lcm_sub = new lcm::LCM(_param);
            _thread = std::thread([]{
                while(true) {
                    if(_lcm_sub->good())
                        _lcm_sub->handle();
                }
            });
        }

        // create LCM publishing object
        if(_lcm_pub == NULL)
            _lcm_pub = new lcm::LCM();
    }
};

#endif // LCM_COMMON_BASE_HPP
