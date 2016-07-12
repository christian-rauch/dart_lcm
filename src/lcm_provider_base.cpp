#include <lcm_provider_base.hpp>

// default construction of static members
lcm::LCM *LCM_CommonBase::_lcm = NULL;
std::thread LCM_CommonBase::_thread;
std::string LCM_CommonBase::_param;

