#ifndef PGSLAM_OPTIMIZER_H
#define PGSLAM_OPTIMIZER_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>

namespace pgslam {

#include "types.h"
#include "MapManager.h"

template <typename T>
class Optimizer {
public:
  using Ptr = std::shared_ptr<Optimizer<T>>;

  using MapManagerPtr = typename MapManager<T>::Ptr;

  IMPORT_PGSLAM_TYPES(T)

  using InputData = std::tuple<Vertex, Vertex, Matrix, CovMatrix>;
  using DataBuffer = std::deque<InputData>;

protected:
  // Variables used to store data being processed
  //! Variable used to store loop closing edges being processed
  DataBuffer data_buffer_;
private:
  //! Factor graph used to build the optimization problem, populated from the graph and the input data
  gtsam::NonlinearFactorGraph factor_graph_;
  //! Initial values for the optimization problem, populated from vertices transformation values
  gtsam::Values initial_values_;
  //! Result of the estimation process
  gtsam::Values current_estimate_;

  //! Pointer to the object to store shared data
  MapManagerPtr map_manager_ptr_;

public:
  Optimizer(MapManagerPtr map_manager_ptr);
  virtual ~Optimizer();

  virtual void AddNewData(Vertex from, Vertex to, const Matrix &T_from_to, const CovMatrix & COV_from_to);

protected:
  void ProcessData();
  virtual void PrepareForOptimization();
  virtual void UpdateAfterOptimization();
  Matrix PmCovToGtsamCov(const Matrix &mat);
  gtsam::Pose3 PmPoseToGtsamPose(const Matrix &mat);
  Matrix GtsamPoseToPmPose(const gtsam::Pose3 &pose);
};

} // pgslam

#include "Optimizer.hpp"

#endif // PGSLAM_OPTIMIZER_H
