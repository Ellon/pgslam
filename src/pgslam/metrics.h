#ifndef PGSLAM_METRICS_H
#define PGSLAM_METRICS_H

namespace pgslam {

template<typename T>
struct Metrics
{
  IMPORT_PGSLAM_TYPES(T)

  static T Distance(const Matrix & T1, const Matrix & T2);
  static T Distance(const Keyframe & K1, const Keyframe & K2);

  static T Weight(const Matrix & T_meas, const CovMatrix & cov_meas);
};

} // pgslam

#include "metrics.hpp"

#endif // PGSLAM_METRICS_H
  