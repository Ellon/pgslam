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
MapManager<T>::MapManager()
{}

template<typename T>
MapManager<T>::~MapManager()
{}

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



  // TODO_ADD_INPUT_TO_LOOP_CLOSING_HERE;

  std::cout << "[MapManager] Added keyframe\n";

  return newkf;
}

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

} // pgslam

#endif // PGSLAM_MAP_MANAGER_HPP
