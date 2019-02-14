#ifndef PGSLAM_LOCALIZER_H
#define PGSLAM_LOCALIZER_H

#include <deque>
#include <thread>
#include <condition_variable>

#include <Eigen/Geometry>

#include "types.h"
#include "MapManager.h"
#include "LocalMap.h"

namespace pgslam {

template<typename T>
class Localizer {
public:
  using Ptr = std::shared_ptr<Localizer<T>>;

  using MapManagerPtr = typename MapManager<T>::Ptr;

  IMPORT_PGSLAM_TYPES(T)

  using LocalMap = pgslam::LocalMap<T>;
  using LocalMapDataBuffer = typename pgslam::LocalMap<T>::DataBuffer;
  using LocalMapComposition = typename pgslam::LocalMap<T>::Composition;

public:
  Localizer(MapManagerPtr map_manager_ptr);
  virtual ~Localizer();

  void SetLocalMapMaxSize(size_t size);
  void SetOverlapThreshold(T overlap_threshold);
  void SetMinimalOverlapThreshold(T minimal_overlap);
  void SetIcpConfig(const std::string &config_path);
  void SetInputFiltersConfig(const std::string &config_path);

  // AddNewData is a virtual method to allow multi-thread handling on the
  // derivated class
  virtual void AddNewData(unsigned long long int timestamp,
                  std::string world_frame_id,
                  Matrix T_world_robot,
                  Matrix T_robot_sensor,
                  DPPtr cloud_ptr);

  std::pair<DP,bool> GetLocalMap();
  std::pair<DP,bool> GetLocalMapInWorldFrame();

  // Method to be called by MapManager after the keyframes get updated with an
  // optimization result
  virtual void UpdateFromGraph();

protected:
  void ProcessData(const Matrix &input_T_world_robot, const Matrix &input_T_robot_sensor, DPPtr input_cloud_ptr);
  // All the following methods are virtual because they access MapManager and
  // need special treatment by the multi-thread version
  virtual void ProcessFirstCloud(DPPtr cloud, const Matrix &T_world_robot);
  virtual void UpdateAfterIcp();

  void UpdateRefkfRobotPose();
  void UpdateWorldRobotPose();
  T    ComputeCurrentOverlap();
  T    ComputeOverlapWith(const LocalMapComposition comp);
  bool IsOverlapEnough(T overlap);
  bool IsBetterComposition(T current_overlap, const LocalMapComposition candidate_comp);
  std::pair<LocalMapComposition,bool> FindNeighborLocalMapComposition();

private:
  // Variables used to store data being processed
  //! Variable used to count number of data input
  size_t count_;
  //! Variable used to store the cloud being currently processed
  DPPtr input_cloud_ptr_;

  // Variables used with data points
  //! Used to transform input clouds from sensor to robot frame
  TransformationPtr rigid_transformation_;
  //! Object that filters all input clouds
  DataPointsFilters input_filters_;
  //! ICP object
  ICPSequence icp_sequence_;
  //! Buffer to hold the icp configuration loaded from yaml file
  std::string icp_config_buffer_;

  //! Object to store shared data (graph of keyframes)
  MapManagerPtr map_manager_ptr_;

  // Variables used to store local and global robot poses
  //! Current robot pose at the current reference keyframe
  Matrix T_refkf_robot_;
  //! Current robot pose at world keyframe
  Matrix T_world_robot_;

  //! Last input robot pose at world keyframe, used to compute delta poses
  Matrix last_input_T_world_robot_;

  //! Buffer of vertexes that compose the next local map.
  LocalMapComposition next_local_map_composition_;
  //! Current local map structure. Contains a DP cloud.
  LocalMap local_map_;

  //! Threshold value used to decide if local maps have enough threshold.
  T overlap_threshold_;
  //! Minimal acceptable overlap value. Overlaps below this value indicates that the transform returned by ICP is of low quality.
  T minimal_overlap_;

};

} // pgslam

#include "Localizer.hpp"

#endif // PGSLAM_LOCALIZER_H
