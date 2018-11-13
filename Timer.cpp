#ifndef PGSLAM_TIMER_CPP
#define PGSLAM_TIMER_CPP

#include "Timer.h"

#include <iostream>

namespace pgslam {

Timer::Timer()
{
  Start();
}

void Timer::Start()
{
  t1_ = std::chrono::high_resolution_clock::now();
}

void Timer::Stop(const std::string &msg) const 
{
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << msg << " took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1_).count()
            << " milliseconds\n";
}

} // pgslam

#endif // PGSLAM_TIMER_CPP
