#ifndef SRC_FLYMS_INCLUDE_FLYMS_READY_CHECK_H_
#define SRC_FLYMS_INCLUDE_FLYMS_READY_CHECK_H_

#include <thread>
#include <atomic>


class ReadyCheck {
 public:
  ReadyCheck();
  ~ReadyCheck();

  int WaitForStartSignal();

 private:
  std::thread read_thread_;
  std::atomic<bool> is_running_;

};

#endif  // SRC_FLYMS_INCLUDE_FLYMS_READY_CHECK_H_
