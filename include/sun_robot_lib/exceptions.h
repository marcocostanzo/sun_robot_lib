#include <stdexcept>

namespace sun {
namespace robot {

class base_exception : public std::exception {
  std::string error_str_;

public:
  base_exception(const std::string &error_str)
      : error_str_(error_str), std::exception() {}
  /** Returns a C-style character string describing the general cause
   *  of the current error.  */
  virtual const char *
  what() const _GLIBCXX_TXN_SAFE_DYN _GLIBCXX_USE_NOEXCEPT override {
    return error_str_.c_str();
  }
};

class ExceededJointLimits : public base_exception {
public:
  ExceededJointLimits(std::string const &msg = "") : base_exception(msg) {}
};

} // namespace robot
} // namespace sun
