#ifndef PGSLAM_LOCALIZER_HPP
#define PGSLAM_LOCALIZER_HPP

#include "Localizer.h"

#include <iostream>
#include <fstream>

#include <chrono> // to compute time spent 

#include "Timer.h"

namespace pgslam {

template<typename T>
Localizer<T>::Localizer(MapManagerPtr map_manager_ptr) :
  stop_{false},
  map_manager_ptr_{map_manager_ptr}
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

  Timer timer;

  // main loop
  while(not stop_) {
    DPPtr cloud_ptr;
    Matrix T_world_robot, T_robot_sensor;
    { // Try to get new data, waits if no data
      std::unique_lock<std::mutex> lock(new_data_mutex_);
      if (new_data_buffer_.empty())
        new_data_cond_var_.wait(lock, [this] {
          return (not this->new_data_buffer_.empty()) or this->stop_;
        });
      // Check for shutdown
      if (stop_) break;
      const InputData & data = new_data_buffer_.front();
      cloud_ptr = data.cloud_ptr;
      T_world_robot = data.T_world_robot;
      T_robot_sensor = data.T_robot_sensor;
      new_data_buffer_.pop_front();
    }

    std::cout << "[Localizer] Processing cloud #" << count << "\n";
    count++;

    // Apply filters to incoming cloud, in scanner frame
    timer.Start();
    input_filters_.apply(*cloud_ptr);
    timer.Stop("[Localizer] Input filters");

    // Put cloud into robot frame
    (*cloud_ptr) = map_manager_ptr_->rigid_transformation_->compute(*cloud_ptr, T_robot_sensor);

    if (not icp_sequence_.hasMap()) {
      map_manager_ptr_->AddFirstKeyframe(cloud_ptr, T_world_robot);
      map_manager_ptr_->UpdateLocalMap();
      {
        auto lock = map_manager_ptr_->LocalMapLock();
        icp_sequence_.setMap(map_manager_ptr_->GetLocalMap());
      }
      // Nothing more to do with this cloud
      continue;
    }

    if (map_manager_ptr_->LocalMapNeedsUpdate()) {
      timer.Start();
      map_manager_ptr_->UpdateLocalMap();
      {
        auto lock = map_manager_ptr_->LocalMapLock();
        icp_sequence_.setMap(map_manager_ptr_->GetLocalMap());
      }
      timer.Stop("[Localizer] Setting new map");
    }

    // If we get here we have a local map, so we can perform ICP

    // Correct robot pose through ICP
    timer.Start();
    Matrix corrected_T_world_robot = icp_sequence_(*cloud_ptr, T_world_robot);
    timer.Stop("[Localizer] ICP");

    T overlap = icp_sequence_.errorMinimizer->getOverlap();
    std::cout << "[Localizer] Current overlap is " << overlap << " \n";
    map_manager_ptr_->AddKeyframeBasedOnOverlap(overlap, cloud_ptr, corrected_T_world_robot);

  } // end main loop
}

} // pgslam

#endif // PGSLAM_LOCALIZER_HPP


