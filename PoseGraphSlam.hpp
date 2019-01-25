#ifndef POSE_GRAPH_SLAM_HPP
#define POSE_GRAPH_SLAM_HPP

#include "PoseGraphSlam.h"

namespace pgslam {

template<typename T>
PoseGraphSlam<T>::PoseGraphSlam()
  : map_manager_ptr_{std::make_shared<MapManager>()},
    optimizer_ptr_{std::make_shared<Optimizer>(map_manager_ptr_)},
    loop_closer_ptr_{std::make_shared<LoopCloser>(map_manager_ptr_, optimizer_ptr_)},
    localizer_ptr_{std::make_shared<Localizer>(map_manager_ptr_)}
{
  // Setup pointers from map manager to main objects
  map_manager_ptr_->SetLoopCloser(loop_closer_ptr_);
}

template<typename T>
PoseGraphSlam<T>::PoseGraphSlam(const std::string & localizer_input_filters_config,
                                const std::string & localizer_icp_config,
                                const std::string & loop_closer_icp_config)
  : PoseGraphSlam()
{
  SetIcpConfig(localizer_input_filters_config, localizer_icp_config, loop_closer_icp_config);
}

template<typename T>
void PoseGraphSlam<T>::SetIcpConfig(const std::string & localizer_input_filters_config,
                                    const std::string & localizer_icp_config,
                                    const std::string & loop_closer_icp_config)
{
  localizer_ptr_->SetInputFiltersConfig(localizer_input_filters_config);
  localizer_ptr_->SetIcpConfig(localizer_icp_config);
  loop_closer_ptr_->SetIcpConfig(loop_closer_icp_config);  
}

template<typename T>
void PoseGraphSlam<T>::AddData(unsigned long long int timestamp,
                               std::string world_frame_id,
                               Matrix T_world_robot,
                               Matrix T_robot_sensor,
                               DPPtr cloud_ptr)
{
  localizer_ptr_->AddNewData(timestamp, world_frame_id, T_world_robot, T_robot_sensor, cloud_ptr);
}

template<typename T>
void PoseGraphSlam<T>::Run()
{
  localizer_ptr_->Run();
  loop_closer_ptr_->Run();
  optimizer_ptr_->Run();
}

} // pgslam

#endif // POSE_GRAPH_SLAM_HPP
