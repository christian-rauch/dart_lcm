#ifndef LCM_SINGELTON_HPP
#define LCM_SINGELTON_HPP

#include <lcm/lcm-cpp.hpp>

/**
 * @brief The LCMSingelton class
 */
class LCMSingelton {
public:
    /**
     * @brief getLCM provides the common lcm object
     * @return lcm object
     */
    static lcm::LCM &getLCM() {
        return *_lcm;
    }

    /**
     * @brief initLCM initialize the lcm object, e.g. calles its constructor.
     *
     * This will destroy the old object and thus, should only be called befor any
     * subscribers or publisher are added. Currently there is no way to add the
     * previous subscribers and publisher again.
     *
     * @param args parameter list of the LCM object constructor
     */
    template<typename... Args>
    static void initLCM(Args&& ...args) {
        delete _lcm;
        _lcm = new lcm::LCM(std::forward<Args>(args)...);
    }

private:
    /**
     * @brief _lcm pointer to common lcm object
     */
    static lcm::LCM *_lcm;
    LCMSingelton() { }
    ~LCMSingelton() { delete _lcm; }
};

/**
 * @brief LCMSingelton::_lcm default instanciated object
 */
lcm::LCM *LCMSingelton::_lcm = new lcm::LCM();

#endif // LCM_SINGELTON_HPP
