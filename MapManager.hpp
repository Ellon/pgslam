#ifndef PGSLAM_MAP_MANAGER_HPP
#define PGSLAM_MAP_MANAGER_HPP

#include "MapManager.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>

namespace pgslam {

using boost::add_vertex;
using boost::num_vertices;
using boost::add_edge;
using boost::graph_traits;
using boost::vertices;


template<typename T>
MapManager<T>::MapManager()// :
  // rigid_transformation_{PM::get().REG(Transformation).create("RigidTransformation")},
  // overlap_range_min_{0.5},
  // overlap_range_max_{0.8},
  // buffer_{3},
  // local_map_{3},
  // next_local_map_composition_{3}
  // local_map_needs_update_{false},
{}

template<typename T>
MapManager<T>::~MapManager()
{}

// template<typename T>
// void MapManager<T>::SetLocalizer(LocalizerPtr localizer_ptr)
// {
//   localizer_wptr_ = localizer_ptr;
// }

// template<typename T>
// MapManager<T>::LocalMapComposition MapManager<T>::GetNextLocalMapComposition()
// {
//   std::unique_lock<std::mutex> lock(graph_mutex_);
//   return next_local_map_composition.ResetFromGraph(graph_, next_local_map_vertices_);
// }

// template<typename T>
// bool MapManager<T>::LocalMapNeedsRebuild()
// {
//   return local_map_.IsOutdated(GetNextLocalMapComposition());
// }

// template<typename T>
// void MapManager<T>::RebuildLocalMap()
// {
//   if (next_local_map_vertices_.empty())
//     throw std::runtime_error("[MapManager] No local maps keyframes when trying to update local map");

//   local_map_.Update(GetNextLocalMapComposition());

//   std::cout << "[MapManager] Rebuilt local map\n";
// }

// template<typename T>
// typename MapManager<T>::DP MapManager<T>::GetLocalMap()
// {
//   return local_map_.Cloud();
// }

// template<typename T>
// typename MapManager<T>::DP MapManager<T>::GetLocalMapInWorldFrame()
// {
//   return local_map_.CloudInWorldFrame();
// }

template<typename T>
std::unique_lock<std::mutex> MapManager<T>::GetGraphLock()
{
  return std::unique_lock<std::mutex>(graph_mutex_);
}

template<typename T>
const typename MapManager<T>::Graph & MapManager<T>::GetGraph()
{
  return graph_;
}

template<typename T>
typename MapManager<T>::Vertex MapManager<T>::AddFirstKeyframe(DPPtr cloud, const Matrix &T_world_kf)
{
  // Add data to graph structure
  Vertex v = add_vertex(graph_);
  graph_[v].id = num_vertices(graph_) - 1;
  graph_[v].cloud_ptr = cloud;
  graph_[v].T_world_kf = T_world_kf;
  graph_[v].optimized_T_world_kf = T_world_kf;
  graph_[v].update_time = std::chrono::high_resolution_clock::now();

  // The first keyframe is the one that will be kept fixed during
  // optimization
  fixed_keyframe_ = v;

  std::cout << "[MapManager] Added first keyframe\n";
  return v;
}

template<typename T>
typename MapManager<T>::LocalMapComposition MapManager<T>::FindLocalMapComposition(size_t capacity, const Matrix & T_world_x)
{
  Vertex v = FindClosestVertex(T_world_x);
  return FindLocalMapComposition(capacity, v);
}

template<typename T>
typename MapManager<T>::LocalMapComposition MapManager<T>::FindLocalMapComposition(size_t capacity, Vertex src)
{
  // Buffer to store distances
  using dist_vec_t = std::vector<T>;
  dist_vec_t distances(num_vertices(graph_));

  // Use Dijkstra to compute distances from src to each vertex
  boost::dijkstra_shortest_paths(graph_, src,
    boost::weight_map(boost::get(&Constraint::weight, graph_))
    .vertex_index_map(boost::get(&Keyframe::id, graph_))
    .distance_map(boost::make_iterator_property_map(distances.begin(),
                                               boost::get(&Keyframe::id, graph_))));

  // Create an aux buffer used to find distance based ordering. Each element
  // will have the Vertex as first and a iterator to respective distance as
  // second.
  using order_elem_t = std::pair<Vertex, typename dist_vec_t::iterator>;
  std::vector<order_elem_t> order(num_vertices(graph_));
  // Fill the aux buffer
  typename graph_traits<Graph>::vertex_iterator vi, vi_end;
  std::tie (vi, vi_end) = vertices(graph_);
  size_t n = 0;
  std::for_each (vi, vi_end, [this, &order, &n, &distances](auto v){
    order[n++] = std::make_pair(v, distances.begin() + this->graph_[v].id);
  });
  // Sort the buffer using the distances
  std::sort (order.begin(), order.end(), [](const auto & a, const auto & b) -> bool {
    return *(a.second) < *(b.second);
  });

  // Build the local map composition
  LocalMapComposition comp(capacity);
  auto order_it = order.begin();
  while (order_it != order.end() and capacity > 0) {
    comp.push_front(order_it->first);
    order_it++;
    capacity--;
  }

  return std::move(comp);
}

template<typename T>
typename MapManager<T>::Vertex MapManager<T>::AddNewKeyframe(Vertex from, const Matrix &T_world_newkf, 
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
  graph_[newkf].id = num_vertices(graph_) - 1;
  graph_[newkf].cloud_ptr = cloud_ptr;
  graph_[newkf].T_world_kf = T_world_newkf;
  graph_[newkf].optimized_T_world_kf = T_world_newkf;
  graph_[newkf].update_time = std::chrono::high_resolution_clock::now();

  // Add new edge to the graph
  Edge e;
  bool success;
  std::tie(e, success) = add_edge(from, newkf, graph_);
  if (not success)
    throw std::logic_error("MapManager<T>::AddNewKeyframe(): Edge from 'from' to 'newkf' already exists in the graph");
  graph_[e].type = Constraint::kOdomConstraint;
  graph_[e].T_from_to = meas_T_from_newkf;
  graph_[e].cov_from_to = meas_cov_from_newkf;
  graph_[e].weight = Weight(meas_T_from_newkf, meas_cov_from_newkf);


  TODO_ADD_INPUT_TO_LOOP_CLOSING_HERE;

  std::cout << "[MapManager] Added keyframe\n";

  return newkf;
}


// template<typename T>
// void MapManager<T>::AddKeyframeBasedOnOverlap(T overlap, DPPtr cloud, const Matrix &T_world_kf)
// {
//   std::unique_lock<std::mutex> lock(graph_mutex_);
//
//   if (overlap < overlap_range_max_) {
//     Vertex v = add_vertex(graph_);
//     graph_[v].cloud_ptr = cloud;
//     graph_[v].T_world_kf = T_world_kf;
//     graph_[v].optimized_T_world_kf = T_world_kf;
//     next_local_map_vertices_.push_back(v);
  
//     // buffer_.push_back(std::make_pair(cloud, T_world_kf));
//     // local_map_needs_update_ = true;
//     last_next_local_map_vertices_update_time_ = std::chrono::high_resolution_clock::now();
//     std::cout << "[MapManager] Added keyframe\n";
//   }
//   if (overlap < overlap_range_min_)
//     std::cerr << "[MapManager] Warning: adding keyframe with overlap below minimum overlap\n";
// }

// template<typename T>
// typename MapManager<T>::Matrix MapManager<T>::GetKeyframeTransform(Vertex v)
// {
//   std::unique_lock<std::mutex> lock(graph_mutex_);
//   return graph_[v].optimized_T_world_kf;
// }

// template<typename T>
// std::pair<typename MapManager<T>::Matrix, typename MapManager<T>::Matrix> MapManager<T>::GetTransformOnKeyframe(Vertex v, const Matrix & T_world_x)
// {
//   Matrix T_world_kf;
//   {
//     std::unique_lock<std::mutex> lock(graph_mutex_);
//     T_world_kf = graph_[v].optimized_T_world_kf; 
//   }
//   Matrix T_kf_x = T_world_kf.inverse() * T_world_x;
//   return std::make_pair(T_world_kf, T_kf_x);
// }

// template<typename T>
// typename MapManager<T>::Matrix MapManager<T>::GetReferenceKeyframeTransform()
// {
//   return local_map_.ReferenceKeyframe().optimized_T_world_kf;
// }

// template<typename T>
// typename MapManager<T>::Vertex MapManager<T>::GetReferenceKeyframeVertex()
// {
//   return local_map_.ReferenceVertex();
// }

// template<typename T>
// typename MapManager<T>::Time MapManager<T>::GetLastKeyframesUpdateTime()
// {
//   return last_kfs_update_time_;
// }

// template<typename T>
// bool MapManager<T>::HasEnoughOverlap(T overlap)
// {
//   if (overlap < overlap_range_min_)
//     std::cerr << "[MapManager] WARNING: overlap below minimum overlap! (" << overlap << " < " << overlap_range_min_ << ")\n";

//   if (overlap < overlap_range_max_)
//     std::cerr << "[MapManager] overlap below threshold! (" << overlap << " < " << overlap_range_max_ << ")\n";
//   else
//     std::cerr << "[MapManager] overlap still ok! (" << overlap << " >= " << overlap_range_max_ << ")\n";

//   return (overlap >= overlap_range_max_);
// }

// template<typename T>
// bool MapManager<T>::FindBetterLocalMap(const Matrix & T_world_x)
// {
//   std::cerr << "MapManager::FindBetterLocalMap not yet implemented!\n";
//   // Find closest vertex

//   // Search in the graph for N close vertices

//   // Build point cloud using keyframe clouds

//   // Check if we have enough overlap with the current cloud

//   return false;
// }


template<typename T>
typename MapManager<T>::Vertex MapManager<T>::FindClosestVertex(const Matrix & T_world_x)
{
  typename graph_traits<Graph>::vertex_iterator vi, vi_end;
  std::tie(vi, vi_end) = vertices(graph_);

  // Compute distance for the first vertex
  Vertex closest_v = *vi;
  T closest_dist = Distance(graph_[closest_v].optimized_T_world_kf, T_world_x);

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

template<typename T>
T MapManager<T>::Weight(const Matrix & T_meas, const CovMatrix & cov_meas)
{
  return T_meas.col(3).head(3).norm();
}


// template<typename T>
// void MapManager<T>::UpdateLocalizerBeforeIcp()
// {
//   if (auto localizer_ptr = localizer_wptr_.lock()) {

//     std::unique_lock<std::mutex> lock(graph_mutex_);

//     // Update world robot pose if refkf pose was updated in the graph
//     if (local_map_.IsReferenceKeyframeOutdated(graph_))
//       localizer_ptr->UpdateWorldRobotPose(graph_);

//     if (not local_map_.HasSameComposition(next_local_map_composition_)) {
//       // Store old reference keyframe and vertex
//       Vertex old_refkf_vertex = local_map_.ReferenceVertex();
//       Time old_refkf_update_time = local_map_.ReferenceKeyframe().update_time;
//       // Update/Rebuild local map
//       local_map_.UpdateToNewComposition(graph_, next_local_map_composition_);
//       // Update local robot pose if needed (different or updated refkf)
//       if (local_map_.ReferenceVertex() != old_refkf_vertex or 
//           local_map_.ReferenceKeyframe().update_time > old_refkf_update_time)
//         localizer_ptr->UpdateLocalRobotPose();

//     } else if (local_map_.IsOutdated(graph_)) {
//       // Store old time
//       Time old_refkf_update_time = local_map_.ReferenceKeyframe().update_time;
//       // Update/Rebuild local map
//       local_map_.UpdateFromGraph(graph_);
//       // Update local robot pose if updated refkf
//       if (local_map_.ReferenceKeyframe().update_time > old_refkf_update_time)
//         localizer_ptr->UpdateLocalRobotPose();
//     }

//   } else {
//     std::cerr << "[MapManager] Weak pointer to Localizer is expired when updating before ICP" << std::endl;
//   }
// }

// template<typename T>
// void MapManager<T>::UpdateLocalizerAfterIcp()
// {
//   if (auto localizer_ptr = localizer_wptr_.lock()) {
//     T overlap = localizer_ptr->ComputeCurrentOverlap();
//     std::cout << "[Localizer] Current overlap is " << overlap << " \n";

//     if (not HasEnoughOverlap(overlap)) {
//       LocalMapComposition better_composition;
//       bool found = false;
//       std::tie(better_composition, found) = FindBetterLocalMapComposition(localizer_ptr->GetWorldRobotPose());
//       if (not found) {
//         Matrix cov_T_refkf_robot = localizer_ptr->GetCurrentCovariance();
//         AddNewKeyframe(refkf_vertex_,
//           T_world_robot_,
//           T_refkf_robot_,
//           cov_T_refkf_robot,
//           input_cloud_ptr);
//         TODO_ADD_INPUT_TO_LOOP_CLOSING_HERE;
//       } else {
//         next_local_map_composition_ = better_composition;
//       }

//     } else if (GetClosestVertex(localizer_ptr->GetWorldRobotPose()) != local_map_.ReferenceVertex()) {
//       LocalMapComposition better_composition;
//       bool found = false;
//       std::tie(better_composition, found) = FindBetterLocalMapComposition(localizer_ptr->GetWorldRobotPose());
//       if (found)
//         next_local_map_composition_ = better_composition;
//     }

//   } else {
//     std::cerr << "[MapManager] Weak pointer to Localizer is expired when updating before ICP" << std::endl;
//   }
// }

// template<typename T>
// std::unique_lock<std::mutex> MapManager<T>::LocalMapLock()
// {
//   return std::unique_lock<std::mutex>{local_map_mutex_};
// }

} // pgslam

#endif // PGSLAM_MAP_MANAGER_HPP
