#ifndef PGSLAM_MAP_MANAGER_HPP
#define PGSLAM_MAP_MANAGER_HPP

#include "MapManager.h"

namespace pgslam {

using boost::add_vertex;
using boost::num_vertices;
using boost::add_edge;
using boost::graph_traits;
using boost::vertices;


template<typename T>
MapManager<T>::MapManager() :
  rigid_transformation_{PM::get().REG(Transformation).create("RigidTransformation")},
  overlap_range_min_{0.5},
  overlap_range_max_{0.8},
  // buffer_{3},
  local_map_composition_{3}
  // local_map_needs_update_{false},
{}

template<typename T>
MapManager<T>::~MapManager()
{
}

template<typename T>
bool MapManager<T>::LocalMapNeedsRebuild()
{
  return (last_local_map_rebuild_time_ < last_kfs_update_time_) or
         (last_local_map_rebuild_time_ < last_local_map_composition_update_time_);
}

template<typename T>
void MapManager<T>::RebuildLocalMap()
{
  if (local_map_composition_.empty())
    throw std::runtime_error("[MapManager] No local maps keyframes when trying to update local map");

  std::unique_lock<std::mutex> lock(local_map_mutex_);

  // NOTE: kf on the back is the reference kf
  // Add reference kf cloud
  auto rit = local_map_composition_.rbegin();
  local_map_ = *(graph_[*rit].cloud_ptr);
  // Get world transform in the reference kf, used below to convert other kf clouds
  Matrix T_refkf_world = graph_[*rit].optimized_T_world_kf.inverse();
  rit++;

  // Convert all other kf clouds in the local map to refkf frame and
  // concatenate in the local map cloud
  std::for_each(rit, local_map_composition_.rend(), [this, &T_refkf_world](const auto &v) {
    this->local_map_.concatenate(this->rigid_transformation_->compute(*(this->graph_[v].cloud_ptr), T_refkf_world * this->graph_[v].optimized_T_world_kf));
  });

  last_local_map_rebuild_time_ = std::chrono::high_resolution_clock::now();
  std::cout << "[MapManager] Rebuilt local map\n";
}

template<typename T>
typename MapManager<T>::DP MapManager<T>::GetLocalMap()
{
  std::unique_lock<std::mutex> lock(local_map_mutex_);
  return local_map_;
}

template<typename T>
typename MapManager<T>::DP MapManager<T>::GetLocalMapInWorldFrame()
{
  std::unique_lock<std::mutex> lock(local_map_mutex_);
  return rigid_transformation_->compute(local_map_, graph_[local_map_composition_.back()].optimized_T_world_kf);
}

template<typename T>
void MapManager<T>::AddFirstKeyframe(DPPtr cloud, const Matrix &T_world_kf)
{
  Vertex v = add_vertex(graph_);
  graph_[v].cloud_ptr = cloud;
  graph_[v].T_world_kf = T_world_kf;
  graph_[v].optimized_T_world_kf = T_world_kf;
  graph_[v].update_time = std::chrono::high_resolution_clock::now();
  fixed_keyframe_ = v;
  local_map_composition_.clear();
  local_map_composition_.push_back(v);

  // buffer_.push_back(std::make_pair(cloud, T_world_kf));
  // local_map_needs_update_ = true;
  last_local_map_composition_update_time_ = std::chrono::high_resolution_clock::now();
  std::cout << "[MapManager] Added first keyframe\n";
}

// template<typename T>
// void MapManager<T>::AddKeyframeBasedOnOverlap(T overlap, DPPtr cloud, const Matrix &T_world_kf)
// {
//   if (overlap < overlap_range_max_) {
//     Vertex v = add_vertex(graph_);
//     graph_[v].cloud_ptr = cloud;
//     graph_[v].T_world_kf = T_world_kf;
//     graph_[v].optimized_T_world_kf = T_world_kf;
//     local_map_composition_.push_back(v);
  
//     // buffer_.push_back(std::make_pair(cloud, T_world_kf));
//     // local_map_needs_update_ = true;
//     last_local_map_composition_update_time_ = std::chrono::high_resolution_clock::now();
//     std::cout << "[MapManager] Added keyframe\n";
//   }
//   if (overlap < overlap_range_min_)
//     std::cerr << "[MapManager] Warning: adding keyframe with overlap below minimum overlap\n";
// }

template<typename T>
typename MapManager<T>::Matrix MapManager<T>::GetKeyframeTransform(Vertex v)
{
  return graph_[v].optimized_T_world_kf;
}

template<typename T>
std::pair<typename MapManager<T>::Matrix, typename MapManager<T>::Matrix> MapManager<T>::GetTransformOnKeyframe(Vertex v, const Matrix & T_world_x)
{
  const Matrix & T_world_kf = graph_[v].optimized_T_world_kf;
  Matrix T_kf_x = T_world_kf.inverse() * T_world_x;
  return std::make_pair(T_world_kf, T_kf_x);
}

template<typename T>
typename MapManager<T>::Matrix MapManager<T>::GetReferenceKeyframe()
{
  return graph_[local_map_composition_.back()].optimized_T_world_kf;
}

template<typename T>
typename MapManager<T>::Vertex MapManager<T>::GetReferenceKeyframeVertex()
{
  return local_map_composition_.back();
}

template<typename T>
typename MapManager<T>::Time MapManager<T>::GetLastKeyframesUpdateTime()
{
  return last_kfs_update_time_;
}

template<typename T>
bool MapManager<T>::HasEnoughOverlap(T overlap)
{
  if (overlap < overlap_range_min_)
    std::cerr << "[MapManager] WARNING: overlap below minimum overlap! (" << overlap << " < " << overlap_range_min_ << ")\n";

  if (overlap < overlap_range_max_)
    std::cerr << "[MapManager] overlap below threshold! (" << overlap << " < " << overlap_range_max_ << ")\n";
  else
    std::cerr << "[MapManager] overlap still ok! (" << overlap << " >= " << overlap_range_max_ << ")\n";

  return (overlap >= overlap_range_max_);
}

template<typename T>
bool MapManager<T>::FindBetterLocalMap(const Matrix & T_world_x)
{
  std::cerr << "MapManager::FindBetterLocalMap not yet implemented!\n";
  // Find closest vertex

  // Search in the graph for N close vertices

  // Build point cloud using keyframe clouds

  // Check if we have enough overlap with the current cloud

  return false;
}

template<typename T>
void MapManager<T>::AddNewKeyframe(Vertex from, const Matrix &T_world_newkf, 
    const Matrix & meas_T_from_newkf, const CovMatrix & meas_cov_from_newkf, 
    DPPtr cloud_ptr)
{
  // Make sure Vertex from exists in the graph
  typename graph_traits<Graph>::vertex_iterator vi, vi_end;
  std::tie(vi, vi_end) = vertices(graph_);
  if (std::find(vi, vi_end, from) == vi_end)
    throw std::logic_error("MapManager<T>::AddNewKeyframe(): Vertex 'from' must exist in the graph");

  // Add new vertex to the graph
  Vertex newkf = add_vertex(graph_);
  graph_[newkf].cloud_ptr = cloud_ptr;
  graph_[newkf].T_world_kf = T_world_newkf;
  graph_[newkf].optimized_T_world_kf = T_world_newkf;

  // Add new edge to the graph
  Edge e;
  bool success;
  std::tie(e, success) = add_edge(from, newkf, graph_);
  graph_[e].type = Constraint::kOdomConstraint;
  graph_[e].T_from_to = meas_T_from_newkf;
  graph_[e].cov_from_to = meas_cov_from_newkf;

  // The new vertex will be used to compose the new local map AND it will be
  // the new reference keyframe (the last element)
  local_map_composition_.push_back(newkf);
  last_local_map_composition_update_time_ = std::chrono::high_resolution_clock::now();

  std::cout << "[MapManager] Added keyframe\n";

}

template<typename T>
typename MapManager<T>::Vertex MapManager<T>::GetClosestKeyframeVertex(const Matrix & T_world_x)
{
  typename graph_traits<Graph>::vertex_iterator vi, vi_end;
  std::tie(vi, vi_end) = vertices(graph_);

  // Compute distance for the first vertex
  Vertex closest_v = *vi;
  T closest_dist = Distance(graph_[local_map_composition_.back()].optimized_T_world_kf, T_world_x);

  // Find the closest vertex
  vi++;
  std::for_each(vi, vi_end, [this, &T_world_x, &closest_v, &closest_dist](Vertex v){
    T dist = Distance(this->graph_[v].optimized_T_world_kf, T_world_x);
    if (dist < closest_dist) {
      closest_v = v;
      closest_dist = dist;
    }
  });

  return closest_v;
}

template<typename T>
T MapManager<T>::Distance(const Matrix & T1, const Matrix & T2)
{
  auto p1 = T1.col(3).head(3);
  auto p2 = T2.col(3).head(3);
  return (p2 - p1).norm();
}


// template<typename T>
// std::unique_lock<std::mutex> MapManager<T>::LocalMapLock()
// {
//   return std::unique_lock<std::mutex>{local_map_mutex_};
// }

} // pgslam

#endif // PGSLAM_MAP_MANAGER_HPP
