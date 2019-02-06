#ifndef PGSLAM_LOOP_CLOSER_HPP
#define PGSLAM_LOOP_CLOSER_HPP

#include "LoopCloser.h"

#include <iostream>

#include <boost/graph/filtered_graph.hpp>

namespace pgslam {

template<typename T>
LoopCloser<T>::LoopCloser(MapManagerPtr map_manager_ptr, OptimizerPtr optimizer_ptr) :
  map_manager_ptr_{map_manager_ptr},
  optimizer_ptr_{optimizer_ptr},
  topo_dist_threshold_{3}, // TODO set this from a parameter
  geom_dist_threshold_{3}, // TODO set this from a parameter
  overlap_threshold_{0.8}, // TODO set this from a parameter
  residual_error_threshold_{5000}, // TODO set this from a parameter
  candidate_local_map_{3}, // TODO set this from a parameter
  input_cloud_ptr_{nullptr}
{}

template<typename T>
LoopCloser<T>::~LoopCloser()
{}

template<typename T>
void LoopCloser<T>::SetIcpConfig(const std::string &config_path)
{
  {
    // Store the yaml as a string so it can be used to create other icp later.
    std::ifstream ifs{config_path, std::ios::ate}; // open the file "at the end"
    auto size = ifs.tellg();
    icp_config_buffer_ = std::string(size, '\0');
    ifs.seekg(0);
    if(not ifs.read(&icp_config_buffer_[0], size))
      throw std::runtime_error("[LoopCloser] Error buffering icp config into a string!");
  }

  // Build stream from buffered string
  std::istringstream iss{icp_config_buffer_};
  icp_.loadFromYaml(iss);
}

template<typename T>
void LoopCloser<T>::AddNewVertex(Vertex v)
{
  ProcessVertex(v);
}

template<typename T>
void LoopCloser<T>::ProcessVertex(Vertex input_vertex)
{
  // Store input vertex internally
  input_vertex_ = input_vertex;

  bool has_candidate = ProcessLocalMapCandidate();
  if (not has_candidate)
    return;

  // Compute the input kf pose in the reference keyframe, that will be
  // the input for the ICP below
  // TODO: Maybe consider only 2D displacements for the input transform?
  Matrix input_T_refkf_kf = candidate_local_map_.ReferenceKeyframe().optimized_T_world_kf.inverse() * input_T_world_kf_;

  // Perform ICP to try to align the input cloud to the candidate local map
  T_refkf_kf_ = icp_(*input_cloud_ptr_, candidate_local_map_.Cloud(), input_T_refkf_kf);

  // Check if ICP succeed
  bool icp_succeed = CheckIcpResult();
  if (icp_succeed) {
    // Add loop closing constraint to the optimizer
    optimizer_ptr_->AddNewData(
      candidate_local_map_.ReferenceVertex(),
      input_vertex,
      T_refkf_kf_,
      icp_.errorMinimizer->getCovariance());
  }
}

template<typename T>
bool LoopCloser<T>::ProcessLocalMapCandidate()
{
  const auto & graph = map_manager_ptr_->GetGraph();

  std::cout << "[LoopCloser] Looking for a loop closing "
            << "candidate for keyframe "
            << graph[input_vertex_].id
            << "\n";

  // If found, the candidate local map will be stored in
  // candidate_local_map_ (an object variable)
  bool candidate_found = FindLocalMapCandidate(input_vertex_);

  // Nothing else to do if we could not find a candidate
  if (not candidate_found) return false;

  // If we get here we have a candidate, so we need to recover input vertex's
  // cloud and pose from the graph
  input_cloud_ptr_ = graph[input_vertex_].cloud_ptr;
  input_T_world_kf_ = graph[input_vertex_].optimized_T_world_kf;

  // Inform that we have a candidate.
  return true;
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
    bool src_suppressed = std::find(suppressed_vertices.begin(), suppressed_vertices.end(), boost::source(e, graph)) != suppressed_vertices.end();
    bool trg_suppressed = std::find(suppressed_vertices.begin(), suppressed_vertices.end(), boost::target(e, graph)) != suppressed_vertices.end();
    return (is_loop_edge or src_suppressed or trg_suppressed);
  });

  // Predicate used to filter the graph. It removes all edges and vertices in
  // both sets above.
  struct Predicate {
    bool operator()(Edge e) const {return std::find(suppressed_edges->begin(), suppressed_edges->end(), e) == suppressed_edges->end(); }
    bool operator()(Vertex v) const {return std::find(suppressed_vertices->begin(), suppressed_vertices->end(), v) == suppressed_vertices->end(); }

    std::vector<Edge> * suppressed_edges;
    std::vector<Vertex> * suppressed_vertices;
  } predicate {&suppressed_edges, &suppressed_vertices};

  // Create the filtered version of the graph
  using Filtered = boost::filtered_graph<Graph, Predicate, Predicate>;
  Filtered filtered_graph(graph, predicate, predicate);

  // Test all candidates
  const auto expected_size = candidate_local_map_.Capacity();
  for (auto candidate_v : candidate_vertices) {
    LocalMapComposition candidate_composition(expected_size);

    // Try to find a local map around the candidate vertex using the filtered
    // graph. The dijkstra call is inside a try-catch to catch the exception
    // throw by the visitor when n vertices are recorded.
    try {
      boost::dijkstra_shortest_paths(filtered_graph, candidate_v,
        boost::weight_map(boost::get(&Constraint::weight, filtered_graph))
        .vertex_index_map(index_map)
        .visitor(boost::make_dijkstra_visitor(
          record_n_and_stop(candidate_composition, expected_size))));
    } catch (StopSearch) {}

    if (candidate_composition.size() == expected_size) {
      candidate_local_map_.UpdateToNewComposition(graph, candidate_composition);

      // DEBUG
      auto print_composition = [](const auto & composition, const auto & index_map){
        std::cout << "(";
        for (auto it = composition.begin(); it != composition.end() - 1; it++)
          std::cout << index_map[*it] << ", ";
        std::cout << index_map[composition.back()] << ")";
      };
      std::cout << "[LoopCloser] Candidate found! -> ";
      print_composition(candidate_composition, index_map);
      std::cout << "\n";

      return true; // Found a candidate. It is stored on candidate_local_map_.
    }
  }

  // DEBUG
  std::cout << "[LoopCloser] Candidate NOT found!\n";

  // Could not find a candidate
  return false;
}

template<typename T>
bool LoopCloser<T>::CheckIcpResult() const
{
  // WARNING: The interface to check for the max number of iterations is not
  // final yet. See https://github.com/ethz-asl/libpointmatcher/pull/314 for
  // details.

  // Check if ICP converged because it reached the max number of allowed
  // iterations
  // NOTE: This is the solution implemented locally
  if (icp_.getMaxNumIterationsReached())
    return false; // Failed

  // NOTE: The commented-out code below is the other way of doing the same as
  // above that is being considered in the mentioned pull request.

  // using CounterTransformationChecker = typename TransformationCheckersImpl<T>::CounterTransformationChecker;
  // for (const auto checker_ptr : icp_.transformationCheckers) {
  //   auto counter_checker_ptr = std::dynamic_pointer_cast<CounterTransformationChecker>(checker_ptr);
  //   if (counter_checker_ptr and counter_tr_checker_ptr->isMaxIterationReached())
  //     return false; // Failed
  // }

  // Check overlap
  if (icp_.errorMinimizer->getOverlap() < overlap_threshold_)
    return false; // Failed

  // Check residual error
  if (ComputeResidualError() > residual_error_threshold_)
    return false; // Failed

  // If we get here, then ICP succeeded.
  return true;
}

template<typename T>
T LoopCloser<T>::ComputeResidualError() const
{
  // Create a temporary icp object
  ICP temp_icp;
  std::istringstream iss{icp_config_buffer_};
  temp_icp.loadFromYaml(iss);

  // Transform input data to express it in ref frame
  DP reading(*input_cloud_ptr_);
  temp_icp.transformations.apply(reading, T_refkf_kf_);

  // To compute residual error:
  // 1) Set reference cloud in the matcher
  temp_icp.matcher->init(candidate_local_map_.Cloud());
  // 2) Get matches between transformed data and ref
  auto matches = temp_icp.matcher->findClosests(reading);
  // 3) Get outlier weights for the matches
  auto outlier_weights = temp_icp.outlierFilters.compute(reading, candidate_local_map_.Cloud(), matches);
  // 4) Compute error
  T residual = temp_icp.errorMinimizer->getResidualError(reading, candidate_local_map_.Cloud(), outlier_weights, matches);

  return residual;
}

} // pgslam

#endif // PGSLAM_LOOP_CLOSER_HPP


