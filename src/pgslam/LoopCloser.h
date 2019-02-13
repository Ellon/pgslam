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
  virtual ~LoopCloser();

  void SetTopologicalDistanceThreshold(T topo_dist_threshold);
  void SetGeometricalDistanceThreshold(T geom_dist_threshold);
  void SetOverlapThreshold(T overlap_threshold);
  void SetResidualErrorThreshold(T residual_error_threshold);
  void SetCandidateLocalMapMaxSize(size_t size);
  void SetIcpConfig(const std::string &config_path);

  virtual void AddNewVertex(Vertex v);

protected:
  virtual bool ProcessLocalMapCandidate();

  void ProcessVertex(Vertex input_vertex);
  bool FindLocalMapCandidate(Vertex input_v);
  bool CheckIcpResult() const;
  T ComputeResidualError() const;

private:
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

  // Variables used to store data being processed
  //! Input vertex to which we try to close a loop
  Vertex input_vertex_;
  //! Input transformation passed to loop closing ICP
  Matrix input_T_world_kf_;
  //! Stores the current point cloud been processed
  DPPtr input_cloud_ptr_;
  //! Stores the resulting transform of ICP with input cloud and candidate local map
  Matrix T_refkf_kf_;

};

} // pgslam

#include "LoopCloser.hpp"

#endif // PGSLAM_LOOP_CLOSER_H
