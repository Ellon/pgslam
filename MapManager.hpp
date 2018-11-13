#ifndef PGSLAM_MAP_MANAGER_HPP
#define PGSLAM_MAP_MANAGER_HPP

#include "MapManager.h"

namespace pgslam {

template<typename T>
MapManager<T>::MapManager() :
  rigid_transformation_{PM::get().REG(Transformation).create("RigidTransformation")},
  overlap_range_min_{0.5},
  overlap_range_max_{0.8},
  buffer_{3},
  local_map_needs_update_{false}
{}

template<typename T>
MapManager<T>::~MapManager()
{
}

template<typename T>
bool MapManager<T>::LocalMapNeedsUpdate()
{
  return local_map_needs_update_;
}

template<typename T>
void MapManager<T>::UpdateLocalMap()
{
  if (buffer_.size() < 1)
    throw std::runtime_error("[MapManager] Buffer empty when trying to get updated local map");
  std::unique_lock<std::mutex> lock(local_map_mutex_);
  auto it = buffer_.begin();
  local_map_ = rigid_transformation_->compute(*(it->first), it->second);
  it++;
  std::for_each(it, buffer_.end(), [this](const auto &cloud_matrix_pair) {
    this->local_map_.concatenate(this->rigid_transformation_->compute(*(cloud_matrix_pair.first), cloud_matrix_pair.second));
  });
  local_map_needs_update_ = false;
  std::cout << "[MapManager] Updated local map\n";
}

template<typename T>
const typename MapManager<T>::DP & MapManager<T>::GetLocalMap()
{
  return local_map_;
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

template<typename T>
std::unique_lock<std::mutex> MapManager<T>::LocalMapLock()
{
  return std::unique_lock<std::mutex>{local_map_mutex_};
}

} // pgslam

#endif // PGSLAM_MAP_MANAGER_HPP
