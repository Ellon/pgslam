#ifndef METRICS_HPP
#define METRICS_HPP

namespace pgslam {

template<typename T>
T Metrics<T>::Distance(const Matrix & T1, const Matrix & T2)
{
  auto p1 = T1.col(3).head(3);
  auto p2 = T2.col(3).head(3);
  return (p2 - p1).norm();
}

template<typename T>
T Metrics<T>::Distance(const Keyframe & K1, const Keyframe & K2)
{
  return Metrics<T>::Distance(K1.optimized_T_world_kf, K2.optimized_T_world_kf);
}

template<typename T>
T Metrics<T>::Weight(const Matrix & T_meas, const CovMatrix & cov_meas)
{
  return T_meas.col(3).head(3).norm();
}

} // pgslam

#endif // METRICS_HPP
