#ifndef PGSLAM_LOCALIZER_MT_HPP
#define PGSLAM_LOCALIZER_MT_HPP

#include "LocalizerMT.h"

namespace pgslam {

template<typename T>
LocalizerMT<T>::LocalizerMT(MapManagerMTPtr map_manager_mt_ptr) :
  Base(map_manager_mt_ptr),
  stop_{false},
  map_manager_mt_ptr_{map_manager_mt_ptr}
{}

template<typename T>
LocalizerMT<T>::~LocalizerMT()
{
  stop_ = true;
  // Threads may be waiting, notify all.
  new_data_cond_var_.notify_all();
  if (main_thread_.joinable())
    main_thread_.join();
}

template<typename T>
void LocalizerMT<T>::AddNewData(unsigned long long int timestamp,
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
void LocalizerMT<T>::Run()
{
  std::cout << "[LocalizerMT] Starting main thread...\n";
  stop_ = false;
  main_thread_ = std::thread(&LocalizerMT<T>::Main, this);
}

template<typename T>
void LocalizerMT<T>::Main()
{
  // main loop
  while(not stop_) {

    // Try to get new input data, waits if no data
    Matrix input_T_world_robot, input_T_robot_sensor;
    DPPtr input_cloud_ptr;
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
               input_cloud_ptr) = new_data_buffer_.front();
      new_data_buffer_.pop_front();
    }

    this->ProcessData(input_T_world_robot, input_T_robot_sensor, input_cloud_ptr);

  } // end main loop
}

template<typename T>
void LocalizerMT<T>::ProcessFirstCloud(DPPtr cloud, const Matrix &T_world_robot)
{
  auto graph_lock = map_manager_mt_ptr_->GetGraphLock();
  Base::ProcessFirstCloud(cloud, T_world_robot);
}

template<typename T>
void LocalizerMT<T>::UpdateBeforeIcp()
{
  auto graph_lock = map_manager_mt_ptr_->GetGraphLock();
  Base::UpdateBeforeIcp();
}

template<typename T>
void LocalizerMT<T>::UpdateAfterIcp()
{
  auto graph_lock = map_manager_mt_ptr_->GetGraphLock();
  Base::UpdateAfterIcp();
}

} // pgslam

#endif // PGSLAM_LOCALIZER_MT_HPP
