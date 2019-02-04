#ifndef PGSLAM_LOOP_CLOSER_H
#define PGSLAM_LOOP_CLOSER_H

#include <deque>
#include <thread>

#include "types.h"
#include "MapManager.h"
#include "Optimizer.h"
#include "LocalMap.h"

namespace pgslam {

template<typename T>
class LoopCloser {
public:
  using Ptr = std::shared_ptr<LoopCloser<T>>;

  using MapManagerPtr = typename MapManager<T>::Ptr;
  using OptimizerPtr = typename Optimizer<T>::Ptr;

  IMPORT_PGSLAM_TYPES(T)

  using LocalMap = pgslam::LocalMap<T>;
  using LocalMapDataBuffer = typename pgslam::LocalMap<T>::DataBuffer;
  using LocalMapComposition = typename pgslam::LocalMap<T>::Composition;

public:
  LoopCloser(MapManagerPtr map_manager_ptr, OptimizerPtr optimizer_ptr);
  ~LoopCloser();

  void SetIcpConfig(const std::string &config_path);

  void AddNewVertex(Vertex v);
  void Run();
  void Main();

private:
  bool FindLocalMapCandidate(Vertex input_v);
  bool CheckIcpResult() const;
  T ComputeResidualError() const;

private:
  // Variables used to input data in the thread
  //! Variable used to stop the thread
  bool stop_ = {false};
  //! Buffer with new data to be processed
  std::deque<Vertex> new_vertex_buffer_;
  //! Mutex to control access to new_vertex_buffer_
  std::mutex new_vertex_mutex_;
  //! Condition variable to inform localization thread of new data
  std::condition_variable new_vertex_cond_var_;
  //! Main thread object
  std::thread main_thread_;

  //! Pointer to the object to store shared data
  MapManagerPtr map_manager_ptr_;

  //! Pointer to the object that performs the optimization
  OptimizerPtr optimizer_ptr_;

  //! Topological distance threshold, below which vertices are considered bad loop closing candidates
  T topo_dist_threshold_;
  //! Geometrical distance threshold, below which vertices are considered potential loop closing candidates
  T geom_dist_threshold_;
  //! Minimum overlap threshold, below which we consider that the loop closing ICP failed
  T overlap_threshold_;
  //! Residual error threshold, used when testing if loop closing ICP converged
  T residual_error_threshold_;

  //! Variable that store the current local map being processed
  LocalMap candidate_local_map_;

  //! Buffer to hold the icp configuration loaded from yaml file
  std::string icp_config_buffer_;
  //! The ICP object
  ICP icp_;

  //! Stores the current point cloud been processed
  DPPtr input_cloud_ptr_;
  //! Stores the resulting transform of ICP with input cloud and candidate local map
  Matrix T_refkf_kf_;

};

} // pgslam

#include "LoopCloser.hpp"

#endif // PGSLAM_LOOP_CLOSER_H
