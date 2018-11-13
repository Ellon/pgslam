#ifndef PGSLAM_MAP_MANAGER_HPP
#define PGSLAM_MAP_MANAGER_HPP

#include "MapManager.h"

namespace pgslam {

template<typename T>
MapManager<T>::MapManager() :
  rigid_transformation_{PM::get().REG(Transformation).create("RigidTransformation")},
  overlap_range_min_{0.5},
  overlap_range_max_{0.75},
  buffer_{3},
  local_map_needs_update_{false}
{}

template<typename T>
bool MapManager<T>::LocalMapNeedsUpdate()
{
  return local_map_needs_update_;
}

template<typename T>
typename MapManager<T>::DP MapManager<T>::GetUpdatedLocalMap()
{
  if (buffer_.size() < 1)
    throw std::runtime_error("[MapManager] Buffer empty when trying to get updated local map");

  auto it = buffer_.begin();
  DP local_map{rigid_transformation_->compute(*(it->first), it->second)};
  it++;

  std::for_each(it, buffer_.end(), [&local_map, this](const auto &cloud_matrix_pair) {
    local_map.concatenate(this->rigid_transformation_->compute(*(cloud_matrix_pair.first), cloud_matrix_pair.second));
  });

  local_map_needs_update_ = false;

  std::cout << "[MapManager] Generated updated local map\n";

  return local_map;
}

template<typename T>
void MapManager<T>::AddFirstKeyframe(DPPtr cloud, const Matrix &T_world_cloud)
{
  buffer_.push_back(std::make_pair(cloud, T_world_cloud));
  local_map_needs_update_ = true;

  std::cout << "[MapManager] Added first keyframe\n";
}

template<typename T>
void MapManager<T>::AddKeyframeBasedOnOverlap(T overlap, DPPtr cloud, const Matrix &T_world_cloud)
{
  if (overlap < overlap_range_max_) {
    buffer_.push_back(std::make_pair(cloud, T_world_cloud));
    local_map_needs_update_ = true;
    std::cout << "[MapManager] Added keyframe\n";
  }

  if (overlap < overlap_range_min_)
    std::cerr << "[MapManager] Warning: adding keyframe with overlap below minimum overlap\n";

}

} // pgslam

#endif // PGSLAM_MAP_MANAGER_HPP
