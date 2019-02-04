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
template<typename T> class LoopCloser;

template<typename T>
class MapManager {
public:
  using Ptr = std::shared_ptr<MapManager<T>>;

  IMPORT_PGSLAM_TYPES(T)

  using LoopCloserPtr = std::shared_ptr<LoopCloser<T>>;
  using LoopCloserWPtr = std::weak_ptr<LoopCloser<T>>;

  using LocalMapComposition = typename pgslam::LocalMap<T>::Composition;

public:
  MapManager(/* args */);
  ~MapManager();

  void SetLoopCloser(LoopCloserWPtr loop_closer_ptr);

  // Getters
  std::unique_lock<std::mutex> GetGraphLock();
  const Graph & GetGraph();
  Vertex GetFixedVertex();

  // Methods to that modify the internal graph
  Vertex AddFirstKeyframe(DPPtr cloud, const Matrix &T_world_kf);
  Vertex AddNewKeyframe(Vertex from, const Matrix &T_world_newkf, 
    const Matrix & meas_T_from_newkf, const CovMatrix & meas_cov_from_newkf, 
    DPPtr cloud_ptr);
  void AddLoopClosingConstraint(Vertex from, Vertex to, const Matrix &T_from_to, const CovMatrix & COV_from_to);
  void UpdateKeyframeTransform(Vertex v, const Matrix &updated_transform);


private:
  //! Graph structure used to store map data
  Graph graph_;
  //! Mutex that controls access to graph_
  std::mutex graph_mutex_;
  //! Vertex that is considered fixed for the optimization.
  Vertex fixed_vertex_;

  //! Weak pointer to loop closer object
  LoopCloserWPtr loop_closer_wptr_;

};

} // pgslam

#include "MapManager.hpp"

#endif // PGSLAM_MAP_MANAGER_H
