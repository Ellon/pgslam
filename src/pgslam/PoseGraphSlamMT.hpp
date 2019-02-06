#ifndef POSE_GRAPH_SLAM_MT_HPP
#define POSE_GRAPH_SLAM_MT_HPP

#include "PoseGraphSlamMT.h"

namespace pgslam {

template<typename T>
PoseGraphSlamMT<T>::PoseGraphSlamMT()
  : Base()
{}

template<typename T>
PoseGraphSlamMT<T>::PoseGraphSlamMT(const std::string & localizer_input_filters_config,
                                const std::string & localizer_icp_config,
                                const std::string & loop_closer_icp_config)
  : Base(localizer_input_filters_config, localizer_icp_config, loop_closer_icp_config)
{}

template<typename T>
void PoseGraphSlamMT<T>::Run()
{
  this->localizer_ptr_->Run();
  this->loop_closer_ptr_->Run();
  this->optimizer_ptr_->Run();
}

} // pgslam

#endif // POSE_GRAPH_SLAM_MT_HPP
