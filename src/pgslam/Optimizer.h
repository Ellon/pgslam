#ifndef OPTIMIZER_H
#define OPTIMIZER_H

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

private:
  // Variables used to input data in the thread
  //! Variable used to stop the thread
  bool stop_ = {false};
  using InputData = std::tuple<Vertex, Vertex, Matrix, CovMatrix>;
  using DataBuffer = std::deque<InputData>;
  DataBuffer new_data_buffer_; //!< Buffer with new data to be processed
  //! Mutex to control access to new_data_buffer_
  std::mutex new_data_mutex_;
  //! Condition variable to inform localization thread of new data
  std::condition_variable new_data_cond_var_;
  //! Main thread object
  std::thread main_thread_;

  //! Pointer to the object to store shared data
  MapManagerPtr map_manager_ptr_;

public:
  Optimizer(MapManagerPtr map_manager_ptr);
  ~Optimizer();

  void AddNewData(Vertex from, Vertex to, const Matrix &T_from_to, const CovMatrix & COV_from_to);
  void Run();
  void Main();

private:
  Matrix PmCovToGtsamCov(const Matrix &mat);
  gtsam::Pose3 PmPoseToGtsamPose(const Matrix &mat);
  Matrix GtsamPoseToPmPose(const gtsam::Pose3 &pose);
};

} // pgslam

#include "Optimizer.hpp"

#endif // OPTIMIZER_H
