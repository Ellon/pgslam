#ifndef PGSLAM_MAP_MANAGER_H
#define PGSLAM_MAP_MANAGER_H

#include <thread>
#include <memory>
#include <condition_variable>

#include <boost/circular_buffer.hpp>

#include "types.h"

namespace pgslam {

template<typename T>
class MapManager {
public:
  using Ptr = std::shared_ptr<MapManager<T>>;

  IMPORT_PGSLAM_TYPES(T)

public:
  MapManager(/* args */);
  ~MapManager();

  bool LocalMapNeedsRebuild();
  void RebuildLocalMap();
  DP GetLocalMap();
  DP GetLocalMapInWorldFrame();

  void AddFirstKeyframe(DPPtr cloud, const Matrix &T_world_kf);
  // void AddKeyframeBasedOnOverlap(T overlap, DPPtr cloud, const Matrix &T_world_kf);

  Matrix GetKeyframeTransform(Vertex v);
  std::pair<Matrix, Matrix> GetTransformOnKeyframe(Vertex v, const Matrix & T_world_x);
  Matrix GetReferenceKeyframe();
  Vertex GetReferenceKeyframeVertex();
  Time GetLastKeyframesUpdateTime();

  Vertex GetClosestKeyframeVertex(const Matrix & T_world_x);

  bool HasEnoughOverlap(T overlap);

  bool FindBetterLocalMap(const Matrix & T_world_x);

  void AddNewKeyframe(Vertex from, const Matrix &T_world_newkf, 
    const Matrix & meas_T_from_newkf, const CovMatrix & meas_cov_from_newkf, 
    DPPtr cloud_ptr);

  // std::unique_lock<std::mutex> LocalMapLock();

private:
  T Distance(const Matrix & T1, const Matrix & T2);

public:
  TransformationPtr rigid_transformation_;

private:
  //! Min value for overlap range. Below this value the user is informed of potential problems or program stops with error
  T overlap_range_min_;
  //! Max value for overlap range. This is used to decide if local maps are good enough.
  T overlap_range_max_;
  // boost::circular_buffer<std::pair<DPPtr, Matrix>> buffer_;
  //! Graph structure used to store map data
  Graph graph_;
  //! Vertex that is considered fixed for the optimization.
  Vertex fixed_keyframe_;
  // bool local_map_needs_update_ = {false};
  //! Point cloud of the current local map, composed when RebuildLocalMap() is called.
  DP local_map_;
  //! Buffer of vertexes that compose the current or next local map. Used by RebuildLocalMap().
  boost::circular_buffer<Vertex> local_map_composition_;
  //! Mutex that controls access to local_map_
  std::mutex local_map_mutex_;
  //! Time of the last update on the keyframe values
  Time last_kfs_update_time_;
  //! Time of the last local map rebuild
  Time last_local_map_rebuild_time_;
  //! Time of the last change on the composition of the local map
  Time last_local_map_composition_update_time_;

};

} // pgslam

#include "MapManager.hpp"

#endif // PGSLAM_MAP_MANAGER_H
