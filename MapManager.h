#ifndef PGSLAM_MAP_MANAGER_H
#define PGSLAM_MAP_MANAGER_H

#include <thread>
#include <memory>
#include <condition_variable>

#include <boost/circular_buffer.hpp>

#include "types.h"
#include "LocalMap.h"

namespace pgslam {

// Prototypes of main classes to be used on weak pointers
template<typename T> class Localizer;

template<typename T>
class MapManager {
public:
  using Ptr = std::shared_ptr<MapManager<T>>;

  IMPORT_PGSLAM_TYPES(T)

  // using LocalizerPtr = std::shared_ptr<Localizer<T>>;
  // using LocalizerWPtr = std::weak_ptr<Localizer<T>>;

  // using LocalMap = pgslam::LocalMap<T>;
  // using LocalMapDataBuffer = typename pgslam::LocalMap<T>::DataBuffer;
  using LocalMapComposition = typename pgslam::LocalMap<T>::CompositionZ;

public:
  MapManager(/* args */);
  ~MapManager();

  std::unique_lock<std::mutex> GetGraphLock();
  const Graph & GetGraph();

  // bool LocalMapNeedsRebuild();
  // void RebuildLocalMap();
  // DP GetLocalMap();
  // DP GetLocalMapInWorldFrame();

  // Methods to set pointers
  // void SetLocalizer(LocalizerPtr localizer_ptr);
  // void SetLoopCloser(LoopCloserPtr loopcloser_ptr);
  // void SetOptimizer(OptimizerPtr optimizer_ptr);

  // Methods used by the Localizer
  Vertex AddFirstKeyframe(DPPtr cloud, const Matrix &T_world_kf);
  LocalMapComposition FindLocalMapComposition(size_t capacity, const Matrix & T_world_x);
  LocalMapComposition FindLocalMapComposition(size_t capacity, Vertex src);
  Vertex AddNewKeyframe(Vertex from, const Matrix &T_world_newkf, 
    const Matrix & meas_T_from_newkf, const CovMatrix & meas_cov_from_newkf, 
    DPPtr cloud_ptr);

  // void UpdateLocalizerBeforeIcp();
  // void UpdateLocalizerAfterIcp();
  // void AddKeyframeBasedOnOverlap(T overlap, DPPtr cloud, const Matrix &T_world_kf);

  // Matrix GetKeyframeTransform(Vertex v);
  // std::pair<Matrix, Matrix> GetTransformOnKeyframe(Vertex v, const Matrix & T_world_x);
  // Matrix GetReferenceKeyframeTransform();
  // Vertex GetReferenceKeyframeVertex();
  // Time GetLastKeyframesUpdateTime();

  Vertex FindClosestVertex(const Matrix & T_world_x);

  // bool HasEnoughOverlap(T overlap);

  // bool FindBetterLocalMap(const Matrix & T_world_x);


  // std::unique_lock<std::mutex> LocalMapLock();

private:
  T Distance(const Matrix & T1, const Matrix & T2);
  T Weight(const Matrix & T_meas, const CovMatrix & cov_meas);
  // LocalMapComposition GetNextLocalMapComposition();

private:
  // TransformationPtr rigid_transformation_;
  //! Min value for overlap range. Below this value the user is informed of potential problems or program stops with error
  // T overlap_range_min_;
  //! Max value for overlap range. This is used to decide if local maps are good enough.
  // T overlap_range_max_;
  // boost::circular_buffer<std::pair<DPPtr, Matrix>> buffer_;
  //! Graph structure used to store map data
  Graph graph_;
  //! Mutex that controls access to graph_
  std::mutex graph_mutex_;
  //! Vertex that is considered fixed for the optimization.
  Vertex fixed_keyframe_;
  //! Current local map structure. Contains a DP cloud.
  // LocalMap local_map_;
  //! Buffer of vertexes that compose the next local map.
  // LocalMapComposition next_local_map_composition_;
  //! Time of the last update on the keyframe values
  // Time last_kfs_update_time_;
  //! Time of the last local map rebuild
  // Time last_local_map_rebuild_time_;
  //! Time of the last change on the composition of the local map
  // Time last_local_map_composition_update_time_;

  //! Weak pointer to the localizer object
  // LocalizerWPtr localizer_wptr_;

};

} // pgslam

#include "MapManager.hpp"

#endif // PGSLAM_MAP_MANAGER_H
