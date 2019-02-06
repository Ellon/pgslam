#ifndef POSE_GRAPH_SLAM_MT_H
#define POSE_GRAPH_SLAM_MT_H

#include <memory>

#include "PoseGraphSlam.h"

#include "MapManagerMT.h"
#include "LocalizerMT.h"
#include "LoopCloserMT.h"
#include "OptimizerMT.h"

namespace pgslam {

/** Multi-thread version of Pose Graph SLAM.
 */
template <typename T>
class PoseGraphSlamMT : public PoseGraphSlamBase<T, MapManagerMT, LocalizerMT, LoopCloserMT, OptimizerMT> {
public:
  using Base = PoseGraphSlamBase<T, MapManagerMT, LocalizerMT, LoopCloserMT, OptimizerMT>;

public:
  PoseGraphSlamMT();

  PoseGraphSlamMT(const std::string & localizer_input_filters_config,
                const std::string & localizer_icp_config,
                const std::string & loop_closer_icp_config);

  void Run();
};

} // pgslam

#include "PoseGraphSlamMT.hpp"

#endif // POSE_GRAPH_SLAM_MT_H
