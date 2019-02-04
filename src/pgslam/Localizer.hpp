#ifndef PGSLAM_LOCALIZER_HPP
#define PGSLAM_LOCALIZER_HPP

#include "Localizer.h"
#include "metrics.h"

#include <iostream>
#include <fstream>

#include <chrono>

#include "Timer.h"

#include <boost/graph/filtered_graph.hpp>

namespace pgslam {

template<typename T>
Localizer<T>::Localizer(MapManagerPtr map_manager_ptr) :
  stop_{false},
  input_cloud_ptr_{nullptr},
  rigid_transformation_{PM::get().REG(Transformation).create("RigidTransformation")},
  map_manager_ptr_{map_manager_ptr},
  T_refkf_robot_{Matrix::Identity(4,4)},
  T_world_robot_{Matrix::Identity(4,4)},
  last_input_T_world_robot_{Matrix::Identity(4,4)},
  next_local_map_composition_{3}, // TODO make this a parameter
  local_map_{3}, // TODO make this a parameter
  overlap_threshold_{0.8}, // TODO make this a parameter
  minimal_overlap_{0.5} // TODO make this a parameter
{}

template<typename T>
Localizer<T>::~Localizer()
{
  stop_ = true;
  // Threads may be waiting, notify all.
  new_data_cond_var_.notify_all();
  if (main_thread_.joinable())
    main_thread_.join();
}

template<typename T>
void Localizer<T>::SetIcpConfig(const std::string &config_path)
{
  {
    // Store the yaml as a string so it can be used to create other
    // icp_sequence later.
    std::ifstream ifs{config_path, std::ios::ate}; // open the file "at the end"
    auto size = ifs.tellg();
    icp_config_buffer_ = std::string(size, '\0');
    ifs.seekg(0);
    if(not ifs.read(&icp_config_buffer_[0], size))
      throw std::runtime_error("[Localizer] Error buffering icp config into a string!");
  }

  // Build stream from buffered string
  std::istringstream iss{icp_config_buffer_};
  icp_sequence_.loadFromYaml(iss);
}

template<typename T>
void Localizer<T>::SetInputFiltersConfig(const std::string &config_path)
{
  std::ifstream ifs(config_path);
  input_filters_ = DataPointsFilters(ifs);
}

template<typename T>
void Localizer<T>::AddNewData(unsigned long long int timestamp,
                              std::string world_frame_id,
                              Matrix T_world_robot,
                              Matrix T_robot_sensor,
                              DPPtr cloud_ptr)
{
  { // Add to buffer
    std::unique_lock<std::mutex> lock(new_data_mutex_);
    new_data_buffer_.push_back(std::make_tuple(timestamp, world_frame_id,
        T_world_robot, T_robot_sensor, cloud_ptr));
  }
  // notify main thread
  new_data_cond_var_.notify_one();
}

template<typename T>
void Localizer<T>::Run()
{
  std::cout << "[Localizer] Starting main thread...\n";
  stop_ = false;
  main_thread_ = std::thread(&Localizer<T>::Main, this);
}

template<typename T>
void Localizer<T>::Main()
{
  unsigned int count = 0;

  Timer timer;

  // main loop
  while(not stop_) {

    // Try to get new input data, waits if no data
    Matrix input_T_world_robot, input_T_robot_sensor;
    {
      std::unique_lock<std::mutex> lock(new_data_mutex_);
      if (new_data_buffer_.empty())
        new_data_cond_var_.wait(lock, [this] {
          return (not this->new_data_buffer_.empty()) or this->stop_;
        });
      // Check for shutdown
      if (stop_) break;
      std::tie(std::ignore,
               std::ignore,
               input_T_world_robot,
               input_T_robot_sensor,
               input_cloud_ptr_) = new_data_buffer_.front();
      new_data_buffer_.pop_front();
    }

    std::cout << "[Localizer] Processing cloud #" << count << "\n";
    count++;

    // Apply input filters to the cloud while it is still in scanner frame. We
    // need to perform it here to have the observation direction vectors
    // pointing to the sensor.
    timer.Start();
    input_filters_.apply(*input_cloud_ptr_);
    timer.Stop("[Localizer] Input filters");

    // Put cloud into robot frame
    (*input_cloud_ptr_) = rigid_transformation_->compute(*input_cloud_ptr_, input_T_robot_sensor);

    // Next block applies only for the first cloud (i.e. when there is no map yet)
    if (not local_map_.HasCloud()) {
      ProcessFirstCloud(input_cloud_ptr_, input_T_world_robot);
      // Store transforms that will be needed on next iteration
      last_input_T_world_robot_ = input_T_world_robot;
      // Nothing more to do with this cloud
      continue;
    }

    // Perform all updates needed before calling ICP
    UpdateBeforeIcp();

    // Compute a delta transform that represents the movement of the robot
    // since last cloud was processed.
    Matrix input_dT_robot = last_input_T_world_robot_.inverse() * input_T_world_robot;

    // Compute the input robot pose in the reference keyframe, that will be
    // the input for the ICP below
    Matrix input_T_refkf_robot = T_refkf_robot_ * input_dT_robot;

    // Correct the input pose through ICP
    timer.Start();
    T_refkf_robot_ = icp_sequence_(*input_cloud_ptr_, input_T_refkf_robot);
    T_world_robot_ = local_map_.ReferenceKeyframe().optimized_T_world_kf * T_refkf_robot_;
    timer.Stop("[Localizer] ICP");

    // Perform all updates needed after the ICP call
    UpdateAfterIcp();

    // Update last pose input for next iteration
    last_input_T_world_robot_ = input_T_world_robot;

  } // end main loop
}

template<typename T>
void Localizer<T>::ProcessFirstCloud(DPPtr cloud, const Matrix &T_world_robot)
{
  auto graph_lock = map_manager_ptr_->GetGraphLock();

  Vertex v = map_manager_ptr_->AddFirstKeyframe(cloud, T_world_robot);
  // Update local map vertices
  next_local_map_composition_.clear();
  next_local_map_composition_.push_back(v);
  // Build the first local map
  // TODO: Better to do this out of the lock
  local_map_.UpdateToNewComposition(map_manager_ptr_->GetGraph(), next_local_map_composition_);
  // Set map on icp object
  icp_sequence_.setMap(local_map_.Cloud());
  // The first keyframe is coincident with the robot
  T_refkf_robot_ = Matrix::Identity(4,4);
  T_world_robot_ = T_world_robot;
}


template<typename T>
void Localizer<T>::UpdateBeforeIcp()
{
  auto graph_lock = map_manager_ptr_->GetGraphLock();
  const auto & graph = map_manager_ptr_->GetGraph();

  // Update world robot pose if refkf pose was updated in the graph
  if (local_map_.IsReferenceKeyframeOutdated(graph)) {
    UpdateWorldRobotPose(graph);
  }

  if (not local_map_.HasSameComposition(next_local_map_composition_)) {
    // Store old reference keyframe and vertex
    Vertex old_refkf_vertex = local_map_.ReferenceVertex();
    Time old_refkf_update_time = local_map_.ReferenceKeyframe().update_time;
    // Update/Rebuild local map
    local_map_.UpdateToNewComposition(graph, next_local_map_composition_);
    // Set map on icp object
    icp_sequence_.setMap(local_map_.Cloud());
    // Update local robot pose if needed (different or updated refkf)
    if (local_map_.ReferenceVertex() != old_refkf_vertex or 
        local_map_.ReferenceKeyframe().update_time > old_refkf_update_time) {
      UpdateRefkfRobotPose(graph);
    }

  } else if (local_map_.IsOutdated(graph)) {
    // Store old time
    Time old_refkf_update_time = local_map_.ReferenceKeyframe().update_time;
    // Update/Rebuild local map
    local_map_.UpdateFromGraph(graph);
    // Set map on icp object
    icp_sequence_.setMap(local_map_.Cloud());
    // Update local robot pose if updated refkf
    if (local_map_.ReferenceKeyframe().update_time > old_refkf_update_time) {
      UpdateRefkfRobotPose(graph);
    }
  }
}

template<typename T>
void Localizer<T>::UpdateAfterIcp()
{
  auto print_composition = [](const auto & composition, const auto & graph){
    auto index_map = boost::get(&Keyframe::id, graph);
    std::cout << "(";
    for (auto it = composition.begin(); it != composition.end() - 1; it++)
      std::cout << index_map[*it] << ", ";
    std::cout << index_map[composition.back()] << ")";
  };

  // Compute current overlap
  T overlap = ComputeCurrentOverlap();
  std::cout << "[Localizer] current overlap = " << overlap << "\n";

  auto graph_lock = map_manager_ptr_->GetGraphLock();

  if (IsOverlapEnough(overlap)) {
    // When the overlap is good enough we won't create a new keyframe on the
    // graph, but still we want to:
    //
    // 1) update the local map to one that is more appropriate if the robot is
    // moving back in the already mapped environment; and
    //
    // 2) have the local map referenced on the vertex that is closest to the
    // current robot position.
    
    // Let's first deal with case #1
    bool neighbor_found = false; 
    LocalMapComposition neighbor_composition;
    std::tie(neighbor_composition, neighbor_found) = FindNeighborLocalMapComposition();
    if (neighbor_found and IsBetterComposition(overlap, neighbor_composition)) {
      next_local_map_composition_ = std::move(neighbor_composition);
    } else {
      // Here we deal with case #2
      auto closest_v = local_map_.FindClosestVertex(T_world_robot_);
      auto ref_v = local_map_.ReferenceVertex();
      if (closest_v != ref_v) {
        next_local_map_composition_ = local_map_.GetComposition();
        auto closest_it = std::find(next_local_map_composition_.begin(), next_local_map_composition_.end(), closest_v);
        auto ref_it = std::find(next_local_map_composition_.begin(), next_local_map_composition_.end(), ref_v);
        std::iter_swap(closest_it, ref_it);
      }
    }
  } else {
    // In this case we most likely want to create a new keyframe, since the
    // case above already keeps the local map to the best one available. But
    // just in case we still look for a better local map here, and if we can't
    // find it we add the new keyframe.
    bool neighbor_found = false; 
    LocalMapComposition neighbor_composition;
    std::tie(neighbor_composition, neighbor_found) = FindNeighborLocalMapComposition();
    if (neighbor_found and IsBetterComposition(overlap, neighbor_composition)) {
      next_local_map_composition_ = std::move(neighbor_composition);
    } else {
      Vertex v = map_manager_ptr_->AddNewKeyframe(
        local_map_.ReferenceVertex(),
        T_world_robot_,
        T_refkf_robot_,
        icp_sequence_.errorMinimizer->getCovariance(),
        input_cloud_ptr_);
      next_local_map_composition_.push_back(v);
      std::cout << "[Localizer] next_local_map_composition_ = ";
      print_composition(next_local_map_composition_, map_manager_ptr_->GetGraph());
      std::cout << "\n";
    }
  }
}

template<typename T>
void Localizer<T>::UpdateRefkfRobotPose(const Graph & g)
{
  T_refkf_robot_ = g[local_map_.ReferenceVertex()].T_world_kf.inverse() * T_world_robot_;
}

template<typename T>
void Localizer<T>::UpdateWorldRobotPose(const Graph & g)
{
  T_world_robot_ = g[local_map_.ReferenceVertex()].T_world_kf * T_refkf_robot_;
}

template<typename T>
T Localizer<T>::ComputeCurrentOverlap()
{
  return icp_sequence_.errorMinimizer->getOverlap();
}

template<typename T>
T Localizer<T>::ComputeOverlapWith(const LocalMapComposition comp)
{
  // To compute the overlap with a arbitrary composition, we need to build a
  // local map from it and check if this local map has enough overlap with the
  // current cloud

  LocalMap temp_local_map{map_manager_ptr_->GetGraph(), comp};

  // We use libpointmatcher's ErrorMinimizer::getOverlap() method to compute
  // the overlap after an ICP (see Localizer<T>::ComputeCurrentOverlap()). It
  // uses ErrorMinimizer's internal variable lastErrorElements that is updated
  // every time we compute the transformation that minimizes the error. If we
  // would follow this path here we would need to perform an ICP between the
  // candidate map and the current input cloud, to then be able to compute the
  // overlap. This is too much if we want only the overlap.
  //
  // The (hackish) way to avoid this is to perform here the same steps ICP
  // would do but only until we get the error elements, then we use the error
  // elements to compute the overlap. This is not the best solution because
  // libpointmatcher's ICP code may evolve in the future, diverging from the
  // lines below.

  using ICP = typename PM::ICP;
  using Matches = typename PM::Matches;
  using OutlierWeights = typename PM::OutlierWeights;
  using ErrorElements = typename PM::ErrorMinimizer::ErrorElements;

  ICP temp_icp;
  std::istringstream iss{icp_config_buffer_};
  temp_icp.loadFromYaml(iss);

  DP reference(temp_local_map.CloudInWorldFrame());
  temp_icp.referenceDataPointsFilters.init();
  temp_icp.referenceDataPointsFilters.apply(reference);

  temp_icp.matcher->init(reference);

  DP reading(*input_cloud_ptr_);
  temp_icp.readingDataPointsFilters.init();
  temp_icp.readingDataPointsFilters.apply(reading);

  reading = rigid_transformation_->compute(reading, T_world_robot_);

  temp_icp.readingStepDataPointsFilters.init();
  temp_icp.readingStepDataPointsFilters.apply(reading);

  const Matches matches(temp_icp.matcher->findClosests(reading));

  const OutlierWeights outlierWeights(temp_icp.outlierFilters.compute(reading, reference, matches));

  ErrorElements matchedPoints(reading, reference, outlierWeights, matches);

  // Here there's another hack going on. To compute the overlap from ICP's
  // ErrorMinimizer object we would need to either:
  //
  //   1) have set it's lastErrorElements variable by performing the ICP, but
  //   we don't want to do that here just to get the overlap;
  //
  //   2) set it by hand or inform the error elements by argument to
  //   ErrorMinimizer::getOverlap(), but the current libpointmatcher interface
  //   does not allow that and we don't want to change it (yet...).
  //
  // Thus as a workaround we're simply using the weightedPointUsedRatio field,
  // that is the default value used by ErrorMinimizer base class.

  return matchedPoints.weightedPointUsedRatio;
}

template<typename T>
bool Localizer<T>::IsOverlapEnough(T overlap)
{
  if (overlap < minimal_overlap_)
    std::cerr << "[Localizer] WARNING: overlap below minimal overlap! (" << overlap << " < " << minimal_overlap_ << ")\n";

  if (overlap < overlap_threshold_)
    std::cout << "[Localizer] overlap below threshold! (" << overlap << " < " << overlap_threshold_ << ")\n";

  return (overlap >= overlap_threshold_);
}

template<typename T>
bool Localizer<T>::IsBetterComposition(T current_overlap, const LocalMapComposition candidate_comp)
{
  // The same composition is not a better composition
  if (local_map_.HasSameComposition(candidate_comp))
    return false;

  T candidate_overlap = ComputeOverlapWith(candidate_comp);

  return IsOverlapEnough(candidate_overlap) and (candidate_overlap > current_overlap);
}

template<typename T>
std::pair<typename Localizer<T>::DP, bool> Localizer<T>::GetLocalMap()
{
  if(local_map_.HasCloud())
    return std::make_pair(local_map_.Cloud(), true);
  else
    return std::make_pair(DP(), false);
}

template<typename T>
std::pair<typename Localizer<T>::DP, bool> Localizer<T>::GetLocalMapInWorldFrame()
{
  if(local_map_.HasCloud())
    return std::make_pair(local_map_.CloudInWorldFrame(), true);
  else
    return std::make_pair(DP(), false);
}

template<typename T>
std::pair<typename Localizer<T>::LocalMapComposition, bool> Localizer<T>::FindNeighborLocalMapComposition()
{
  // Find set of vertices that are adjacent to current local map
  std::set<Vertex> map_adj_vs;
  auto curr_comp = local_map_.GetComposition();
  const auto & graph = map_manager_ptr_->GetGraph();
  std::for_each(curr_comp.begin(), curr_comp.end(), [&map_adj_vs, &curr_comp, &graph](auto comp_v) {
    auto adj_vs = boost::adjacent_vertices(comp_v, graph);
    std::for_each(adj_vs.first, adj_vs.second, [&map_adj_vs, &curr_comp](auto adj_v){
      if (std::find(curr_comp.begin(), curr_comp.end(), adj_v) == curr_comp.end())
        map_adj_vs.insert(adj_v);
    });
  });

  // Return here if we could not find adjacent vertices
  if (map_adj_vs.empty())
    return std::make_pair(LocalMapComposition(), false);

  // Find the vertex in the adjacency set that is closest to robot
  auto map_adj_vs_it = map_adj_vs.begin();
  auto closest_adj_v = *map_adj_vs_it;
  map_adj_vs_it++;
  auto closest_dist = Metrics<T>::Distance(graph[closest_adj_v].optimized_T_world_kf, T_world_robot_);
  std::for_each(map_adj_vs_it, map_adj_vs.end(), [this, &closest_adj_v, &closest_dist, &graph](auto v) {
    auto dist = Metrics<T>::Distance(graph[v].optimized_T_world_kf, this->T_world_robot_);
    if (dist < closest_dist) {
      closest_dist = dist;
      closest_adj_v = v;
    }
  });

  // Create a container with the current composition plus closest adjacent
  // vertex (we call it "extended composition")
  std::vector<Vertex> curr_comp_ext(curr_comp.begin(), curr_comp.end());
  curr_comp_ext.push_back(closest_adj_v);

  // Create a filtered graph with local map, the closest adjacent vertex and
  // all edges between them. The predicate below is used to create this
  // filtered graph
  struct Predicate {
    bool operator()(Edge e) const {return (std::find(vertices_to_keep->begin(), vertices_to_keep->end(), boost::source(e, *graph)) != vertices_to_keep->end() and
                                           std::find(vertices_to_keep->begin(), vertices_to_keep->end(), boost::target(e, *graph)) != vertices_to_keep->end()); }
    bool operator()(Vertex v) const {return std::find(vertices_to_keep->begin(), vertices_to_keep->end(), v) != vertices_to_keep->end(); }

    std::vector<Vertex> * vertices_to_keep;
    const Graph * graph;
  } predicate {&curr_comp_ext, &graph};
  using Filtered = boost::filtered_graph<Graph, Predicate, Predicate>;
  Filtered filtered_graph(graph, predicate, predicate);

  // Compute the topological distance from the closest adjacent vertex in the
  // filtered graph
  std::vector<T> topo_dists(boost::num_vertices(filtered_graph));
  auto index_map = boost::get(&Keyframe::id, filtered_graph);
  auto topo_map = boost::make_iterator_property_map(topo_dists.begin(), index_map);
  boost::dijkstra_shortest_paths(filtered_graph, closest_adj_v,
    boost::weight_map(boost::get(&Constraint::weight, filtered_graph))
    .vertex_index_map(index_map)
    .distance_map(topo_map));

  // Sort the extended composition by DECREASING topological distance (note
  // the reverse iterators)
  std::sort(curr_comp_ext.rbegin(), curr_comp_ext.rend(), [&topo_map](auto v1, auto v2) -> bool {
    return topo_map[v1] < topo_map[v2];
  });

  // Create the neighbor composition WITH ALL BUT THE LAST TWO of the extended
  // composition. This is OK because in the worst case of a local map
  // composition of 1 element, we would still have an extended composition of
  // 2 elements, thus nothing would be copied.
  LocalMapComposition neighbor_composition(local_map_.Capacity());
  std::copy(curr_comp_ext.begin(), curr_comp_ext.end()-2, std::back_inserter(neighbor_composition));

  // Finally add the two last elements in a way that the one closest to the
  // robot stays at the end (this will be the reference frame). Note that his
  // will push the first vertex that was added above (most distant one) out of
  // the neighbor composition, due to it's circular buffer nature
  auto last = curr_comp_ext.rbegin();
  auto last_dist = Metrics<T>::Distance(graph[*last].optimized_T_world_kf, T_world_robot_);
  auto before_last = curr_comp_ext.rbegin()+1;
  auto before_last_dist = Metrics<T>::Distance(graph[*before_last].optimized_T_world_kf, T_world_robot_);
  if (before_last_dist < last_dist) {
    neighbor_composition.push_back(*last);
    neighbor_composition.push_back(*before_last);
  } else {
    neighbor_composition.push_back(*before_last);
    neighbor_composition.push_back(*last);    
  }

  return std::make_pair(std::move(neighbor_composition), true);
}


} // pgslam

#endif // PGSLAM_LOCALIZER_HPP


