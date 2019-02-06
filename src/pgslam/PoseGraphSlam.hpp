#ifndef PGSLAM_POSE_GRAPH_SLAM_HPP
#define PGSLAM_POSE_GRAPH_SLAM_HPP

#include "PoseGraphSlam.h"

namespace pgslam {

template<typename T,
  template <typename> class MapManagerClass,
  template <typename> class LocalizerClass,
  template <typename> class LoopCloserClass,
  template <typename> class OptimizerClass>
PoseGraphSlamBase<T, MapManagerClass, LocalizerClass, LoopCloserClass, OptimizerClass>::PoseGraphSlamBase()
  : map_manager_ptr_{std::make_shared<MapManager>()},
    optimizer_ptr_{std::make_shared<Optimizer>(map_manager_ptr_)},
    loop_closer_ptr_{std::make_shared<LoopCloser>(map_manager_ptr_, optimizer_ptr_)},
    localizer_ptr_{std::make_shared<Localizer>(map_manager_ptr_)}
{
  // Setup pointers from map manager to main objects
  map_manager_ptr_->SetLoopCloser(loop_closer_ptr_);
}

template<typename T,
  template <typename> class MapManagerClass,
  template <typename> class LocalizerClass,
  template <typename> class LoopCloserClass,
  template <typename> class OptimizerClass>
PoseGraphSlamBase<T, MapManagerClass, LocalizerClass, LoopCloserClass, OptimizerClass>::PoseGraphSlamBase(
  const std::string & localizer_input_filters_config,
  const std::string & localizer_icp_config,
  const std::string & loop_closer_icp_config)
  : PoseGraphSlamBase()
{
  SetIcpConfig(localizer_input_filters_config, localizer_icp_config, loop_closer_icp_config);
}

template<typename T,
  template <typename> class MapManagerClass,
  template <typename> class LocalizerClass,
  template <typename> class LoopCloserClass,
  template <typename> class OptimizerClass>
void PoseGraphSlamBase<T, MapManagerClass, LocalizerClass, LoopCloserClass, OptimizerClass>::SetIcpConfig(
  const std::string & localizer_input_filters_config,
  const std::string & localizer_icp_config,
  const std::string & loop_closer_icp_config)
{
  localizer_ptr_->SetInputFiltersConfig(localizer_input_filters_config);
  localizer_ptr_->SetIcpConfig(localizer_icp_config);
  loop_closer_ptr_->SetIcpConfig(loop_closer_icp_config);  
}

template<typename T,
  template <typename> class MapManagerClass,
  template <typename> class LocalizerClass,
  template <typename> class LoopCloserClass,
  template <typename> class OptimizerClass>
void PoseGraphSlamBase<T, MapManagerClass, LocalizerClass, LoopCloserClass, OptimizerClass>::AddData(
  unsigned long long int timestamp,
  std::string world_frame_id,
  Matrix T_world_robot,
  Matrix T_robot_sensor,
  DPPtr cloud_ptr)
{
  localizer_ptr_->AddNewData(timestamp, world_frame_id, T_world_robot, T_robot_sensor, cloud_ptr);
}

template<typename T,
  template <typename> class MapManagerClass,
  template <typename> class LocalizerClass,
  template <typename> class LoopCloserClass,
  template <typename> class OptimizerClass>
void PoseGraphSlamBase<T, MapManagerClass, LocalizerClass, LoopCloserClass, OptimizerClass>::WriteGraphviz(const std::string & path)
{
  map_manager_ptr_->WriteGraphviz(path);
}

} // pgslam

#endif // PGSLAM_POSE_GRAPH_SLAM_HPP
