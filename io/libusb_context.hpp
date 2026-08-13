#ifndef IO__LIBUSB_CONTEXT_HPP
#define IO__LIBUSB_CONTEXT_HPP

#include <libusb-1.0/libusb.h>
#include <mutex>

namespace io
{
class LibusbContext
{
public:
  void acquire()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (refs_++ == 0) {
      libusb_init(nullptr);
    }
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (refs_ > 0 && --refs_ == 0) {
      libusb_exit(nullptr);
    }
  }

private:
  std::mutex mutex_;
  int refs_{0};
};

inline LibusbContext & libusb_context()
{
  static LibusbContext ctx;
  return ctx;
}

}  // namespace io

#endif  // IO__LIBUSB_CONTEXT_HPP
