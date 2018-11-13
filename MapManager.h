#ifndef PGSLAM_MAP_MANAGER_H
#define PGSLAM_MAP_MANAGER_H

#include <thread>
#include <condition_variable>

#include <boost/circular_buffer.hpp>

#include <pointmatcher/PointMatcher.h>

namespace pgslam {

template<typename T>
class MapManager {
public:
  using Ptr = std::shared_ptr<MapManager<T>>;

  using PM = PointMatcher<T>;
  using DP = typename PM::DataPoints;
  using DPPtr = std::shared_ptr<DP>;
  using Matrix = typename PM::Matrix;
  using ICPSequence = typename PM::ICPSequence;
  using TransformationPtr = std::shared_ptr<typename PM::Transformation>;
  
public:
  MapManager(/* args */);
  ~MapManager();

  bool LocalMapNeedsUpdate();
  void UpdateLocalMap();
  const DP & GetLocalMap();

  void AddFirstKeyframe(DPPtr cloud, const Matrix &T_world_cloud);
  void AddKeyframeBasedOnOverlap(T overlap, DPPtr cloud, const Matrix &T_world_cloud);

  std::unique_lock<std::mutex> LocalMapLock();

public:
  TransformationPtr rigid_transformation_;

private:
  T overlap_range_min_, overlap_range_max_;
  boost::circular_buffer<std::pair<DPPtr, Matrix>> buffer_;
  bool local_map_needs_update_ = {false};
  DP local_map_;
  //! Mutex that controls access to local_map
  std::mutex local_map_mutex_;

};

} // pgslam

#include "MapManager.hpp"

#endif // PGSLAM_MAP_MANAGER_H
