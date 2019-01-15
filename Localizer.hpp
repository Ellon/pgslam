#ifndef PGSLAM_LOCALIZER_HPP
#define PGSLAM_LOCALIZER_HPP

#include "Localizer.h"

#include <iostream>
#include <fstream>

#include <chrono>

#include "Timer.h"

namespace pgslam {

template<typename T>
Localizer<T>::Localizer(MapManagerPtr map_manager_ptr) :
  stop_{false},
  input_cloud_ptr_{nullptr},
  rigid_transformation_{PM::get().REG(Transformation).create("RigidTransformation")},
  map_manager_ptr_{map_manager_ptr},
  T_world_refkf_{Matrix::Identity(4,4)},
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
void Localizer<T>::SetLocalIcpConfig(const std::string &config_path)
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
void Localizer<T>::AddNewData(const InputData & data)
{
  { // Add to buffer
    std::unique_lock<std::mutex> lock(new_data_mutex_);
    new_data_buffer_.push_back(data);
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

  // default constructor initializes tyme to epoch
  // typename MapManager<T>::Time last_refkf_reading_time;

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
      const InputData & data = new_data_buffer_.front();
      input_cloud_ptr_ = data.cloud_ptr;
      input_T_world_robot = data.T_world_robot;
      input_T_robot_sensor = data.T_robot_sensor;
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

    // Next block applies only for the first cloud (i.e. when icp has no map yet)
    if (not icp_sequence_.hasMap()) {
      // NOTE: this will set the icp_sequence_ map
      ProcessFirstCloud(input_cloud_ptr_, input_T_world_robot);
      // Store transforms that will be needed on next iteration
      last_input_T_world_robot_ = input_T_world_robot;
      // Nothing more to do with this cloud
      continue;
    }

    // Perfom all updates needed before calling ICP
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
    T_world_robot_ = T_world_refkf_ * T_refkf_robot_;
    timer.Stop("[Localizer] ICP");

    // Perfom all updates needed after the ICP call
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
  T_world_refkf_ = T_world_robot;
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
    UpdateWorldRefkfPose(graph);
    UpdateWorldRobotPose(graph);
  }

  if (not local_map_.HasSameComposition(next_local_map_composition_)) {
    // Store old reference keyframe and vertex
    Vertex old_refkf_vertex = local_map_.ReferenceVertex();
    Time old_refkf_update_time = local_map_.ReferenceKeyframe().update_time;
    // Update/Rebuild local map
    local_map_.UpdateToNewComposition(graph, next_local_map_composition_);
    // Update local robot pose if needed (different or updated refkf)
    if (local_map_.ReferenceVertex() != old_refkf_vertex or 
        local_map_.ReferenceKeyframe().update_time > old_refkf_update_time) {
      UpdateWorldRefkfPose(graph);
      UpdateLocalRobotPose(graph);
    }

  } else if (local_map_.IsOutdated(graph)) {
    // Store old time
    Time old_refkf_update_time = local_map_.ReferenceKeyframe().update_time;
    // Update/Rebuild local map
    local_map_.UpdateFromGraph(graph);
    // Update local robot pose if updated refkf
    if (local_map_.ReferenceKeyframe().update_time > old_refkf_update_time) {
      UpdateWorldRefkfPose(graph);
      UpdateLocalRobotPose(graph);
    }
  }
}

template<typename T>
void Localizer<T>::UpdateAfterIcp()
{
  // Compute current overlap
  T overlap = icp_sequence_.errorMinimizer->getOverlap();

  auto graph_lock = map_manager_ptr_->GetGraphLock();

  if (not IsOverlapEnough(overlap)) {
    LocalMapComposition composition_candidate = map_manager_ptr_->FindLocalMapComposition(local_map_.Capacity(), T_world_robot_);
    if (not IsBetterComposition(overlap, composition_candidate)) {
      Vertex v = map_manager_ptr_->AddNewKeyframe(
        local_map_.ReferenceVertex(),
        T_world_robot_,
        T_refkf_robot_,
        icp_sequence_.errorMinimizer->getCovariance(),
        input_cloud_ptr_);
      next_local_map_composition_.push_back(v);
    } else {
      next_local_map_composition_ = std::move(composition_candidate);
    }

    TODO_FIND_CLOSEST_VERTEX_ONLY_INSIDE_LOCAL_MAP_OR_NOT_QUESTION_MARK;
  } else if (map_manager_ptr_->FindClosestVertex(T_world_robot_) != local_map_.ReferenceVertex()) {
    LocalMapComposition composition_candidate = map_manager_ptr_->FindLocalMapComposition(local_map_.Capacity(), T_world_robot_);
    if (IsBetterComposition(overlap, composition_candidate))
      next_local_map_composition_ = std::move(composition_candidate);
  }
}

template<typename T>
void Localizer<T>::UpdateWorldRefkfPose(const Graph & g)
{
  T_world_refkf_ = g[local_map_.ReferenceVertex()].T_world_kf;
}

template<typename T>
void Localizer<T>::UpdateWorldRobotPose(const Graph & g)
{
  T_world_robot_ = g[local_map_.ReferenceVertex()].T_world_kf * T_refkf_robot_;
}

template<typename T>
void Localizer<T>::UpdateLocalRobotPose(const Graph & g)
{
  T_refkf_robot_ = g[local_map_.ReferenceVertex()].T_world_kf.inverse() * T_world_robot_;
}

template<typename T>
bool Localizer<T>::IsOverlapEnough(T overlap)
{
  if (overlap < minimal_overlap_)
    std::cerr << "[Localizer] WARNING: overlap below minimal overlap! (" << overlap << " < " << minimal_overlap_ << ")\n";

  return (overlap >= overlap_threshold_);
}

template<typename T>
bool Localizer<T>::IsBetterComposition(T current_overlap, const LocalMapComposition candidade_comp)
{
  // The same composition is not a better composition
  if (local_map_.HasSameComposition(candidade_comp))
    return false;

  // If composition is different, then we need to build a local map from it
  // and check if this local map has enough overlap with the current cloud

  LocalMap candidate_local_map{map_manager_ptr_->GetGraph(), candidade_comp};

  // Normally the overlap computed by ICP object's getOverlap() uses
  // ErrorMinimizer's variable lastErrorElements that is updated every time we
  // compute the transformation that minimizes the error. If we would follow
  // this path here we would need to perform an ICP between the candidate map
  // and the current input cloud, to then be able to compute the overlap. This
  // is too much if we want only the overlap.
  //
  // The (hackish) way to avoid this is to perform here the same steps ICP
  // would do but only until we get the error elements, then we use the error
  // elements to compute the overlap. This is not the best solution because
  // libpointmatcher's ICP code may evolve in the future, mismatching the
  // lines below.

  using ICP = typename PM::ICP;
  using Matches = typename PM::Matches;
  using OutlierWeights = typename PM::OutlierWeights;
  using ErrorElements = typename PM::ErrorMinimizer::ErrorElements;

  ICP temp_icp;
  std::istringstream iss{icp_config_buffer_};
  temp_icp.loadFromYaml(iss);

  DP reference(candidate_local_map.CloudInWorldFrame());
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

  T candidate_overlap = matchedPoints.weightedPointUsedRatio;

  return IsOverlapEnough(candidate_overlap) and (candidate_overlap > current_overlap);
}

template<typename T>
std::pair<typename Localizer<T>::DP, bool> Localizer<T>::GetLocalMap()
{
  if(icp_sequence_.hasMap())
    return std::make_pair(icp_sequence_.getPrefilteredMap(), true);
  else
    return std::make_pair(DP(), false);
}

template<typename T>
std::pair<typename Localizer<T>::DP, bool> Localizer<T>::GetLocalMapInWorldFrame()
{
  if(icp_sequence_.hasMap())
    return std::make_pair(rigid_transformation_->compute(icp_sequence_.getPrefilteredMap(), T_world_refkf_), true);
  else
    return std::make_pair(DP(), false);
}

} // pgslam

#endif // PGSLAM_LOCALIZER_HPP


