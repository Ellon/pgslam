#ifndef PGSLAM_TIMER_H
#define PGSLAM_TIMER_H

#include <string>
#include <chrono>

namespace pgslam {

class Timer {
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> t1_;
public:
  Timer();
  void Start();
  void Stop(const std::string &msg) const;
};

} // pgslam

#endif // PGSLAM_TIMER_H
