#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <deque>
#include <thread>
#include <condition_variable>

#include <Eigen/Geometry>

#include "MapManager.h"

namespace pgslam {

template<typename T>
class Localizer {
public:
  using PM = typename MapManager<T>::PM;
  using DP = typename MapManager<T>::DP;
  using DPPtr = typename MapManager<T>::DPPtr;
  using Matrix = typename MapManager<T>::Matrix;
  using ICPSequence = typename MapManager<T>::ICPSequence;
  using DataPointsFilters = typename PM::DataPointsFilters;
  using TransformationPtr = typename MapManager<T>::TransformationPtr;
  using MapManagerPtr = typename MapManager<T>::Ptr;

  Localizer(MapManagerPtr map_manager_ptr);
  ~Localizer();

  void SetLocalIcpConfig(const std::string &config_path);
  void SetInputFiltersConfig(const std::string &config_path);

  void AddNewData(const DPPtr cloud, const Matrix &T_world_robot, const Matrix &T_robot_sensor);
  void Run();
  void Main();

private:
  //! 
  bool stop_ = {false};
  //! Buffer with new data to be processed
  std::deque<std::tuple<DPPtr, Matrix, Matrix>> new_data_buffer_;
  //! Mutex to control access to new_data_buffer_
  std::mutex new_data_mutex_;
  //! Condition variable to inform localization thread of new data
  std::condition_variable new_data_cond_var_;
  //! Main thread object
  std::thread main_thread_;
  //! Object that filters all input clouds
  DataPointsFilters input_filters_;
  //! ICP object
  ICPSequence icp_sequence_;
  //! Object to store shared data
  MapManagerPtr map_manager_ptr_;
};

} // pgslam

#include "Localizer.hpp"

#endif // LOCALIZER_H
