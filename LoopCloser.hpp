#ifndef PGSLAM_LOOP_CLOSER_HPP
#define PGSLAM_LOOP_CLOSER_HPP

#include "LoopCloser.h"

#include <iostream>

#include <boost/graph/filtered_graph.hpp>

namespace pgslam {

template<typename T>
LoopCloser<T>::LoopCloser(MapManagerPtr map_manager_ptr) :
  stop_{false},
  map_manager_ptr_{map_manager_ptr},
  topo_dist_threshold_{3}, // TODO set this from a parameter
  geom_dist_threshold_{3}, // TODO set this from a parameter
  candidade_local_map_{3} // TODO set this from a parameter
{}

template<typename T>
LoopCloser<T>::~LoopCloser()
{
  stop_ = true;
  // Threads may be waiting, notify all.
  new_vertex_cond_var_.notify_all();
  if (main_thread_.joinable())
    main_thread_.join();
}

template<typename T>
void LoopCloser<T>::AddNewVertex(Vertex v)
{
  { // Add to buffer
    std::unique_lock<std::mutex> lock(new_vertex_mutex_);
    new_vertex_buffer_.push_back(v);
  }
  // notify main thread
  new_vertex_cond_var_.notify_one();
}

template<typename T>
void LoopCloser<T>::Run()
{
  std::cout << "[LoopCloser] Starting main thread...\n";
  stop_ = false;
  main_thread_ = std::thread(&LoopCloser<T>::Main, this);
}

template<typename T>
void LoopCloser<T>::Main()
{
  auto print_composition = [](const auto & composition, const auto & graph){
    auto index_map = boost::get(&Keyframe::id, graph);
    std::cout << "(";
    for (auto it = composition.begin(); it != composition.end() - 1; it++)
      std::cout << index_map[*it] << ", ";
    std::cout << index_map[composition.back()] << ")";
  };

  // main loop
  while(not stop_) {

    // Try to get new input vertex, waits if no vertex
    Vertex input_vertex;
    {
      std::unique_lock<std::mutex> lock(new_vertex_mutex_);
      if (new_vertex_buffer_.empty())
        new_vertex_cond_var_.wait(lock, [this] {
          return (not this->new_vertex_buffer_.empty()) or this->stop_;
        });
      // Check for shutdown
      if (stop_) break;
      input_vertex = new_vertex_buffer_.front();
      new_vertex_buffer_.pop_front();
    }

    {
      auto graph_lock = map_manager_ptr_->GetGraphLock();

      std::cout << "[LoopCloser] Looking for a loop closing "
                << "candidate for keyframe "
                << map_manager_ptr_->GetGraph()[input_vertex].id
                << "\n";

      bool found_candidate = FindLocalMapCandidate(input_vertex);

      if (found_candidate) {
        std::cout << "[LoopCloser] Candidate found! -> ";
        print_composition(candidade_local_map_.GetComposition(), map_manager_ptr_->GetGraph());
        std::cout << "\n";
      } else {
        std::cout << "[LoopCloser] Candidate NOT found!\n";
      }

    }
  }
}

// Visitor used to compute geometric distance to one reference vertex
template <class DistanceMap>
struct geom_dist_computer : public boost::base_visitor<geom_dist_computer<DistanceMap>> {

  typedef boost::on_discover_vertex event_filter;

  inline geom_dist_computer(typename DistanceMap::key_type v, DistanceMap d_map)
  : v_ref_(v), distance_map_(d_map) {}
  
  template <class Vertex, class Graph>
  void operator()(Vertex v, Graph& graph) {
    distance_map_[v] = Metrics<typename DistanceMap::value_type>::Distance(graph[v], graph[v_ref_]);
  }

private:
  typename DistanceMap::key_type v_ref_;
  DistanceMap distance_map_;
};
template <class DistanceMap>
inline geom_dist_computer<DistanceMap>
compute_geom_dist(typename DistanceMap::key_type v, DistanceMap d_map) {
  typedef geom_dist_computer<DistanceMap> GDC;
  return GDC(v, d_map);
}

// Predicate function object used to filter edges
// template <class ToFilterOutContainer>
// struct filter_out_t {
//   using Constraint = typename Types<T>::Constraint;
//   filter_out_t() { }
//   filter_out_t(const ToFilterOutContainer & to_filter_out)
//   : m_to_filter_out(to_filter_out) { }
//   template <typename Edge>
//   bool operator()(const Edge& e) const {
//     bool is_odom_edge = boost::get(m_type_map, e) == Constraint::kOdomConstraint;
//     bool src_not_in_no_loop = std::find(m_no_loop.begin(), m_no_loop.end(), boost::source(e, *m_graph_ptr)) == m_no_loop.end();
//     bool trg_not_in_no_loop = std::find(m_no_loop.begin(), m_no_loop.end(), boost::target(e, *m_graph_ptr)) == m_no_loop.end();
//     return (is_odom_edge and src_not_in_no_loop and trg_not_in_no_loop);
//   }
// private:
//   ToFilterOutContainer m_no_loop;
// };

// Type thrown by the visitor below to stop the search.
struct StopSearch {};

// Visitor used to store N first examined vertex in a container.
template <class VertexContainer>
struct n_and_stop_recorder : public boost::base_visitor<n_and_stop_recorder<VertexContainer>> {
  typedef boost::on_examine_vertex event_filter;

  inline n_and_stop_recorder(VertexContainer &container, size_t n) 
  : container_(container), n_(n) {} 

  template <class Vertex, class Graph>
  void operator()(Vertex v, Graph&) {
    container_.push_front(v);
    if(container_.size() >= n_)
      throw StopSearch();
  }

private:
  VertexContainer & container_;
  size_t n_;
};
template <class VertexContainer>
inline n_and_stop_recorder<VertexContainer>
record_n_and_stop(VertexContainer & container, size_t n) {
  typedef n_and_stop_recorder<VertexContainer> NASR;
  return NASR(container, n);
}

template<typename T>
bool LoopCloser<T>::FindLocalMapCandidate(Vertex input_v)
{
  const auto & graph = map_manager_ptr_->GetGraph();

  assert(boost::num_vertices(graph) > 1);

  // Buffers to store distances
  std::vector<T> topo_dists(boost::num_vertices(graph));
  std::vector<T> geom_dists(boost::num_vertices(graph));

  // property maps used in many parts below
  auto index_map = boost::get(&Keyframe::id, graph);
  auto topo_map = boost::make_iterator_property_map(topo_dists.begin(), index_map);
  auto geom_map = boost::make_iterator_property_map(geom_dists.begin(), index_map);
  auto edge_type_map = boost::get(&Constraint::type, graph);

  // Use Dijkstra to compute topological distances from input_v to each other
  // vertex, while also computing the geometrical distances using a visitor.
  boost::dijkstra_shortest_paths(graph, input_v,
    boost::weight_map(boost::get(&Constraint::weight, graph))
    .vertex_index_map(index_map)
    .distance_map(topo_map)
    .visitor(boost::make_dijkstra_visitor(compute_geom_dist(input_v, geom_map))));

  // {
  //   // DEBUG
  //   typename boost::graph_traits<Graph>::vertex_iterator vi, vi_end;
  //   std::tie (vi, vi_end) = boost::vertices(graph);
  //   std::for_each (vi, vi_end, [&graph, &index_map, &topo_map, &geom_map](auto v){
  //     std::cout << "Vertex: " << index_map[v] 
  //               << ", topo_dist: " << topo_map[v]
  //               << ", geom_dist:  " << geom_map[v]
  //               << "\n";
  //   });  
  // }

  // Define a set of possible loop closing vertices. They should be both
  // geometrically close and topologically far from the input vertex. We use
  // thresholds to define if a vertex is close or far.
  std::vector<Vertex> candidate_vertices;
  const auto vs = boost::vertices(graph);
  std::copy_if(vs.first, vs.second, std::back_inserter(candidate_vertices), [this, &topo_map, &geom_map](auto v) -> bool {
    return (geom_map[v] <= this->geom_dist_threshold_) and (topo_map[v] > this->topo_dist_threshold_);
  });

  // Sort candidates by geometrical distance
  std::sort(candidate_vertices.begin(), candidate_vertices.end(), [&geom_map](auto v1, auto v2) -> bool {
    return geom_map[v1] < geom_map[v2];
  });

  // Define a vertex set with the vertices that cannot be in the used to
  // create the local map because they are topologically too close. Vertices
  // in this set will be out of the filtered graph below.
  std::vector<Vertex> suppressed_vertices;
  std::copy_if(vs.first, vs.second, std::back_inserter(suppressed_vertices), [this, &topo_map](auto v) -> bool {
    return topo_map[v] <= this->topo_dist_threshold_;
  });

  // Define a edge set that should be out of the filtered graph below. It
  // contains all the loop edges but also edges connecting at least one vertex
  // set to be filtered out.
  std::vector<Edge> suppressed_edges;
  const auto es = boost::edges(graph);
  std::copy_if(es.first, es.second, std::back_inserter(suppressed_edges), [&graph, &edge_type_map, &suppressed_vertices](auto e) -> bool {
    using Constraint = typename Types<T>::Constraint;
    bool is_loop_edge = boost::get(edge_type_map, e) == Constraint::kLoopConstraint;
    bool src_filtered_out = std::find(suppressed_vertices.begin(), suppressed_vertices.end(), boost::source(e, graph)) != suppressed_vertices.end();
    bool trg_filtered_out = std::find(suppressed_vertices.begin(), suppressed_vertices.end(), boost::target(e, graph)) != suppressed_vertices.end();
    return (is_loop_edge or src_filtered_out or trg_filtered_out);
  });

  // Create the filtered version of the graph, removing all edges and vertices
  // in both sets above.
  struct Predicate {
    bool operator()(Edge e) const {return std::find(suppressed_edges->begin(), suppressed_edges->end(), e) == suppressed_edges->end(); }
    bool operator()(Vertex v) const {return std::find(suppressed_vertices->begin(), suppressed_vertices->end(), v) == suppressed_vertices->end(); }

    std::vector<Edge> * suppressed_edges;
    std::vector<Vertex> * suppressed_vertices;
  } predicate {&suppressed_edges, &suppressed_vertices};

  using Filtered = boost::filtered_graph<Graph, Predicate, Predicate>;
  Filtered filtered_graph(graph, predicate, predicate);

  // // Define a no loop buffer with the vertices whose topo_dist is equal or below
  // // a certain threshold
  // std::vector<Vertex> no_loop_vec;
  // const auto vs = vertices(graph);
  // std::copy_if(vs.first, vs.second, std::back_inserter(no_loop_vec), [this, &topo_map](auto v) -> bool {
  //   return topo_map[v] <= this->topo_dist_threshold_;
  // });

  // // Order vertices by geometrical distance
  // std::vector<Vertex> geom_dist_ordered_vertices(vs.first, vs.second);
  // std::sort(geom_dist_ordered_vertices.begin(), geom_dist_ordered_vertices.end(), [&geom_map](auto v1, auto v2) -> bool {
  //   return geom_map[v1] < geom_map[v2];
  // });

  // {
  //   // DEBUG
  //   std::cout << "vertices:  ";
  //   std::for_each (geom_dist_ordered_vertices.begin(), geom_dist_ordered_vertices.end(), [&index_map](auto v){
  //     std::cout << index_map[v] << " ";
  //   });
  //   std::cout << "\n";
  //   std::cout << "distances: ";
  //   std::for_each (geom_dist_ordered_vertices.begin(), geom_dist_ordered_vertices.end(), [&geom_map](auto v){
  //     std::cout << geom_map[v] << " ";
  //   });
  //   std::cout << "\n";    
  // }

  // Create a odometry tree from the graph, i.e. a graph containing only the
  // odometry edges that is a tree by construction.
  // edge_filter_t<decltype(edges_to_filter_out)> edge_filter(edges_to_filter_out);
  // auto edge_filter = [&edge_type_map, &no_loop_vertices](Edge e, Graph &g) -> bool {
  //   using Constraint = typename Types<T>::Constraint;
  //   bool is_odom_edge = boost::get(edge_type_map, e) == Constraint::kOdomConstraint;
  //   bool src_not_in_no_loop = std::find(no_loop_vertices.begin(), no_loop_vertices.end(), boost::source(e, g)) == no_loop_vertices.end();
  //   bool trg_not_in_no_loop = std::find(no_loop_vertices.begin(), no_loop_vertices.end(), boost::target(e, g)) == no_loop_vertices.end();
  //   return (is_odom_edge and src_not_in_no_loop and trg_not_in_no_loop);
  // };
  // auto vertex_filter = [&no_loop_vertices](Vertex v, Graph &){
  //   // True if v is not on no-loop set
  //   return (std::find(no_loop_vertices.begin(), no_loop_vertices.end(), v) == no_loop_vertices.end());
  // };
  // boost::filtered_graph<Graph, decltype(edge_filter)> filtered_graph(graph, edge_filter);
  // boost::filtered_graph<Graph, decltype(edge_filter), decltype(vertex_filter)> filtered_graph(graph, edge_filter, vertex_filter);

  // auto topo_dist_greater_than_threshold = [this, &topo_map](Vertex v) -> bool { return topo_map[v] > this->topo_dist_threshold_; };

  // // Test all vertices that are below a certain geom dist threshold
  // for (auto vi = geom_dist_ordered_vertices.begin(); vi != geom_dist_ordered_vertices.end() and geom_map[*vi] <= geom_dist_threshold_; vi++) {
  //   // Do not test the input vertex
  //   if (*vi == input_v) continue;

  //   // Find N-1 neighbors of the vertex that can be used in the local map candidate
  //   boost::dijkstra_shortest_paths(filtered_graph, *vi,
  //     boost::weight_map(boost::get(&Constraint::weight, filtered_graph))
  //     .vertex_index_map(index_map)
  //     // .distance_map(topo_map)
  //     .visitor(boost::make_dijkstra_visitor(std::make_pair(
  //       record_n_if(candidate_vertices, candidade_local_map_.capacity(), topo_dist_greater_than_threshold),
  //       )))
  //     );
  // }

  const auto expected_size = candidade_local_map_.Capacity();
  for (auto candidate_v : candidate_vertices) {
    LocalMapComposition candidate_composition(expected_size);

    // Try to find a local map around the candidate vertex using the odometry
    // tree. The dijkstra call is inside a try-catch to catch the exception
    // throw by the visitor when n vertices are recorded.
    try {
      boost::dijkstra_shortest_paths(filtered_graph, candidate_v,
        boost::weight_map(boost::get(&Constraint::weight, filtered_graph))
        .vertex_index_map(index_map)
        .visitor(boost::make_dijkstra_visitor(
          record_n_and_stop(candidate_composition, expected_size))));
    } catch (StopSearch) {}

    if (candidate_composition.size() == expected_size) {
      candidade_local_map_.UpdateToNewComposition(graph, candidate_composition);
      return true; // Found a candidate. It is stored on candidade_local_map_.
    }
  }

  // Could not find a candidate
  return false;
}

} // pgslam

#endif // PGSLAM_LOOP_CLOSER_HPP


