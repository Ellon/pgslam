#ifndef PGSLAM_LOCALIZER_H
#define PGSLAM_LOCALIZER_H

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
  using Vertex = typename MapManager<T>::Vertex;

public:
  struct InputData {
    unsigned long long int timestamp;
    std::string world_frame_id;
    Matrix T_world_robot;
    Matrix T_robot_sensor;
    DPPtr cloud_ptr;
  };

public:
  Localizer(MapManagerPtr map_manager_ptr);
  ~Localizer();

  void SetLocalIcpConfig(const std::string &config_path);
  void SetInputFiltersConfig(const std::string &config_path);

  void AddNewData(const InputData &data);
  void Run();
  void Main();

  std::pair<DP,bool> GetLocalMap();
  std::pair<DP,bool> GetLocalMapInWorldFrame();

private:
  //! 
  bool stop_ = {false};
  //! Buffer with new data to be processed
  std::deque<InputData> new_data_buffer_;
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
  //! Current reference keyframe at the world frame
  Matrix T_world_refkf_;
  //! Current robot pose at the current reference keyframe
  Matrix T_refkf_robot_;
  //! Current robot pose at world keyframe
  Matrix T_world_robot_;
  //! Last input robot pose at world keyframe, used to compute delta poses
  Matrix last_input_T_world_robot_;
  //! The graph vertex of the reference frame in the graph
  Vertex refkf_vertex_;
};

} // pgslam

#include "Localizer.hpp"

#endif // PGSLAM_LOCALIZER_H
