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
        if(_lcm == NULL) {
            _param = param;
        }
        else {
            std::cerr<<"LCM object is already instiated. Calling init() will have no effect."<<std::endl;
        }
    }

protected:
    LCM_CommonBase() {
        start();
    }

    ~LCM_CommonBase() { delete _lcm; }

    /**
     * @brief getLCM provides the common lcm object
     * @return lcm object
     */
    static lcm::LCM &getLCM() {
        return *_lcm;
    }

private:
    static lcm::LCM *_lcm;      /// LCM object pointer
    static std::string _param;  /// LCM provider string
    static std::thread _thread; /// thread

    /**
     * @brief start instantiate LCM and run handle thread
     */
    void start() {
        if(_lcm == NULL) {
            _lcm = new lcm::LCM(_param);
            _thread = std::thread([]{
                while(true) {
                    if(_lcm->good())
                        _lcm->handle();
                }
            });
        }
    }
};

// default construction of static members
lcm::LCM *LCM_CommonBase::_lcm = NULL;
std::thread LCM_CommonBase::_thread;
std::string LCM_CommonBase::_param;

#endif // LCM_COMMON_BASE_HPP
