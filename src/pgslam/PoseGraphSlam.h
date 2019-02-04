#ifndef POSE_GRAPH_SLAM_H
#define POSE_GRAPH_SLAM_H

#include <memory>

#include "types.h"
#include "MapManager.h"
#include "Localizer.h"
#include "LoopCloser.h"
#include "Optimizer.h"

namespace pgslam {

template<typename T>
class PoseGraphSlam
{
public:
  // Aliases for pgslam main classes
  using MapManager = pgslam::MapManager<T>;
  using MapManagerPtr = typename MapManager::Ptr;
  using Localizer = pgslam::Localizer<T>;
  using LocalizerPtr = typename Localizer::Ptr;
  using LoopCloser = pgslam::LoopCloser<T>;
  using LoopCloserPtr = typename LoopCloser::Ptr;
  using Optimizer = pgslam::Optimizer<T>;
  using OptimizerPtr = typename Optimizer::Ptr;

  IMPORT_PGSLAM_TYPES(T)

protected:
  MapManagerPtr map_manager_ptr_;
  OptimizerPtr optimizer_ptr_;
  LoopCloserPtr loop_closer_ptr_;
  LocalizerPtr localizer_ptr_;

public:
  PoseGraphSlam();

  PoseGraphSlam(const std::string & localizer_input_filters_config,
                const std::string & localizer_icp_config,
                const std::string & loop_closer_icp_config);

  void SetIcpConfig(const std::string & localizer_input_filters_config,
                    const std::string & localizer_icp_config,
                    const std::string & loop_closer_icp_config);

  void AddData(unsigned long long int timestamp,
              std::string world_frame_id,
              Matrix T_world_robot,
              Matrix T_robot_sensor,
              DPPtr cloud_ptr);

  void Run();
};

} // pgslam

#include "PoseGraphSlam.hpp"

#endif // POSE_GRAPH_SLAM_H
