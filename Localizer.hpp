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
  map_manager_ptr_{map_manager_ptr},
  T_world_refkf_{Matrix::Identity(4,4)},
  T_refkf_robot_{Matrix::Identity(4,4)},
  T_world_robot_{Matrix::Identity(4,4)},
  last_input_T_world_robot_{Matrix::Identity(4,4)}
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
  std::ifstream ifs(config_path);
  icp_sequence_.loadFromYaml(ifs);
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
  typename MapManager<T>::Time last_refkf_reading_time;

  Timer timer;

  // main loop
  while(not stop_) {

    // Try to get new input data, waits if no data
    DPPtr input_cloud_ptr;
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
      input_cloud_ptr = data.cloud_ptr;
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
    input_filters_.apply(*input_cloud_ptr);
    timer.Stop("[Localizer] Input filters");

    // Put cloud into robot frame
    (*input_cloud_ptr) = map_manager_ptr_->rigid_transformation_->compute(*input_cloud_ptr, input_T_robot_sensor);

    // Next block applies only for the first cloud (i.e. when icp has no map yet)
    if (not icp_sequence_.hasMap()) {
      map_manager_ptr_->AddFirstKeyframe(input_cloud_ptr, input_T_world_robot);
      map_manager_ptr_->RebuildLocalMap();
      icp_sequence_.setMap(map_manager_ptr_->GetLocalMap());
      // Store the descriptor of the current reference kf
      refkf_vertex_ = map_manager_ptr_->GetReferenceKeyframeVertex();
      // Store transforms that will be needed on next iteration
      T_world_robot_ = input_T_world_robot;
      last_input_T_world_robot_ = input_T_world_robot;
      std::tie(T_world_refkf_, T_refkf_robot_) = map_manager_ptr_->GetTransformOnKeyframe(refkf_vertex_, input_T_world_robot);
      // Store the time we reference kf was read
      last_refkf_reading_time = std::chrono::high_resolution_clock::now();
      // Nothing more to do with this cloud
      continue;
    }

    // TODO: Be sure all data races are properly handled!

    // Reference keyframe pose value may have changed by another thread
    if (last_refkf_reading_time < map_manager_ptr_->GetLastKeyframesUpdateTime()) {
      T_world_refkf_ = map_manager_ptr_->GetKeyframeTransform(refkf_vertex_);
      last_refkf_reading_time = std::chrono::high_resolution_clock::now();
      T_world_robot_ = T_world_refkf_ * T_refkf_robot_;
    }

    // Local map may need to be rebuilt due to changes caused by another
    // thread.
    if (map_manager_ptr_->LocalMapNeedsRebuild()) {
      timer.Start();
      map_manager_ptr_->RebuildLocalMap();
      icp_sequence_.setMap(map_manager_ptr_->GetLocalMap());
      timer.Stop("[Localizer] Setting new map");

      // NOTE: We consider that the only thread allowed to perform an
      // operation that would change the list of keyframes composing the local
      // map (by triggering a search for a better local map or by adding a new
      // keyframe) is this thread. That is why we don't check if the reference
      // keyframe vertex has changed here. This case is handled if and when
      // the local map changes, after the ICP calls below.

    }

    // If we get here we have an updated local map, so we can perform ICP

    // Compute a delta transform that represents the movement of the robot
    // since last cloud was processed.
    Matrix input_dT_robot = last_input_T_world_robot_.inverse() * input_T_world_robot;

    // Compute the input robot pose in the reference keyframe, that will be
    // the input for the ICP below
    Matrix input_T_refkf_robot = T_refkf_robot_ * input_dT_robot;

    // Correct the input pose through ICP
    timer.Start();
    T_refkf_robot_ = icp_sequence_(*input_cloud_ptr, input_T_refkf_robot);
    T_world_robot_ = T_world_refkf_ * T_refkf_robot_;
    timer.Stop("[Localizer] ICP");

    T overlap = icp_sequence_.errorMinimizer->getOverlap();
    std::cout << "[Localizer] Current overlap is " << overlap << " \n";

    // Here we inspect the robot pose and the overlap to take decisions about
    // the current local map and the addition of keyframes
    bool local_map_changed = false;
    if (not map_manager_ptr_->HasEnoughOverlap(overlap)) {
      bool found = map_manager_ptr_->FindBetterLocalMap(T_world_robot_);
      if (not found) {
        Matrix cov_T_refkf_robot = icp_sequence_.errorMinimizer->getCovariance();
        map_manager_ptr_->AddNewKeyframe(refkf_vertex_, T_world_robot_, T_refkf_robot_, cov_T_refkf_robot, input_cloud_ptr);
      }
      // In either case (found better local map or added new kf) the local map has changed
      local_map_changed = true;
    } else if (refkf_vertex_ != map_manager_ptr_->GetClosestKeyframeVertex(T_world_robot_)) {
      bool found = map_manager_ptr_->FindBetterLocalMap(T_world_robot_);
      if (found) local_map_changed = true;
    }

    // Perform local map rebuild if needed.
    if (local_map_changed) {
      map_manager_ptr_->RebuildLocalMap();
      icp_sequence_.setMap(map_manager_ptr_->GetLocalMap());

      // The vertex reference keyframe may also have changed. If that's the
      // case we need to update the robot pose wrt. the new reference
      // keyframe, but without changing the robot pose wrt. world frame
      if (refkf_vertex_ != map_manager_ptr_->GetReferenceKeyframeVertex()) {
        refkf_vertex_ = map_manager_ptr_->GetReferenceKeyframeVertex();
        std::tie(T_world_refkf_, T_refkf_robot_) = map_manager_ptr_->GetTransformOnKeyframe(refkf_vertex_, T_world_robot_);
        last_refkf_reading_time = std::chrono::high_resolution_clock::now();
      }
    }

    // Update last pose input for next iteration
    last_input_T_world_robot_ = input_T_world_robot;

  } // end main loop
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
    return std::make_pair(map_manager_ptr_->rigid_transformation_->compute(icp_sequence_.getPrefilteredMap(), T_world_refkf_), true);
  else
    return std::make_pair(DP(), false);
}


} // pgslam

#endif // PGSLAM_LOCALIZER_HPP


