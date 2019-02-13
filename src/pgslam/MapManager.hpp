#ifndef PGSLAM_MAP_MANAGER_HPP
#define PGSLAM_MAP_MANAGER_HPP

#include "MapManager.h"
#include "metrics.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>

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
const typename MapManager<T>::Graph & MapManager<T>::GetGraph()
{
  return graph_;
}

template<typename T>
void MapManager<T>::SetLoopCloser(LoopCloserPtr loop_closer_ptr)
{
  loop_closer_wptr_ = loop_closer_ptr;
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
  fixed_vertex_ = v;

  std::cout << "[MapManager] Added first keyframe\n";
  return v;
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
  graph_[e].weight = Metrics<T>::Weight(meas_T_from_newkf, meas_cov_from_newkf);

  // std::cout << "[MapManager DEBUG]  graph_[e].weight = " <<  graph_[e].weight << '\n';

  if (auto loop_closer_ptr = loop_closer_wptr_.lock()) {
    loop_closer_ptr->AddNewVertex(newkf);
  } else {
    std::cerr << "[MapManager] loop_closer_wptr_ is expired\n";
  }

  std::cout << "[MapManager] Added keyframe " << graph_[newkf].id << "\n";

  return newkf;
}

template<typename T>
typename MapManager<T>::Vertex MapManager<T>::GetFixedVertex()
{
  return fixed_vertex_;
}


template<typename T>
void MapManager<T>::AddLoopClosingConstraint(Vertex from, Vertex to, const Matrix &T_from_to, const CovMatrix & COV_from_to)
{
  // Add new edge to the graph
  Edge e;
  bool success;
  std::tie(e, success) = add_edge(from, to, graph_);
  if (not success)
    throw std::logic_error("MapManager<T>::AddLoopClosingConstraint(): Edge from 'from' to 'to' already exists in the graph");
  graph_[e].type = Constraint::kLoopConstraint;
  graph_[e].T_from_to = T_from_to;
  graph_[e].cov_from_to = COV_from_to;
  graph_[e].weight = Metrics<T>::Weight(T_from_to, COV_from_to);
}

template<typename T>
void MapManager<T>::UpdateKeyframeTransform(Vertex v, const Matrix &updated_transform)
{
  graph_[v].optimized_T_world_kf = updated_transform;
}

template<typename T>
void MapManager<T>::WriteGraphviz(const std::string & path)
{
  // Open file for writting
  std::ofstream ofs(path);

  boost::dynamic_properties dp;
  dp.property("node_id", boost::get(&Keyframe::id, graph_));
  dp.property("label", boost::get(&Keyframe::id, graph_));

  boost::write_graphviz_dp(ofs, graph_, dp);
}


} // pgslam

#endif // PGSLAM_MAP_MANAGER_HPP
