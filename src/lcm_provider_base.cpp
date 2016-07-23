#include <lcm_provider_base.hpp>

// default construction of static members
lcm::LCM *LCM_CommonBase::_lcm_sub = NULL;
lcm::LCM *LCM_CommonBase::_lcm_pub = NULL;
std::thread LCM_CommonBase::_thread;
std::string LCM_CommonBase::_param;
bool LCM_CommonBase::_exit_failure = false;
