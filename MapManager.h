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

  using LocalMapComposition = typename pgslam::LocalMap<T>::Composition;

public:
  MapManager(/* args */);
  ~MapManager();

  std::unique_lock<std::mutex> GetGraphLock();
  const Graph & GetGraph();

  // Methods used by the Localizer
  Vertex AddFirstKeyframe(DPPtr cloud, const Matrix &T_world_kf);
  LocalMapComposition FindLocalMapComposition(size_t capacity, const Matrix & T_world_x);
  LocalMapComposition FindLocalMapComposition(size_t capacity, Vertex src);
  Vertex AddNewKeyframe(Vertex from, const Matrix &T_world_newkf, 
    const Matrix & meas_T_from_newkf, const CovMatrix & meas_cov_from_newkf, 
    DPPtr cloud_ptr);
  Vertex FindClosestVertex(const Matrix & T_world_x);

private:
  T Distance(const Matrix & T1, const Matrix & T2);
  T Weight(const Matrix & T_meas, const CovMatrix & cov_meas);

private:
  //! Graph structure used to store map data
  Graph graph_;
  //! Mutex that controls access to graph_
  std::mutex graph_mutex_;
  //! Vertex that is considered fixed for the optimization.
  Vertex fixed_keyframe_;

};

} // pgslam

#include "MapManager.hpp"

#endif // PGSLAM_MAP_MANAGER_H
