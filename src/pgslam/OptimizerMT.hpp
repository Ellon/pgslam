#ifndef PGSLAM_OPTIMIZER_MT_HPP
#define PGSLAM_OPTIMIZER_MT_HPP

#include "OptimizerMT.h"

namespace pgslam {

template<typename T>
OptimizerMT<T>::OptimizerMT(MapManagerMTPtr map_manager_mt_ptr) :
  Base(map_manager_mt_ptr),
  stop_{false},
  map_manager_mt_ptr_{map_manager_mt_ptr}
{}

template<typename T>
OptimizerMT<T>::~OptimizerMT()
{
  stop_ = true;
  // Threads may be waiting, notify all.
  new_data_cond_var_.notify_all();
  if (main_thread_.joinable())
    main_thread_.join();
}

template<typename T>
void OptimizerMT<T>::AddNewData(Vertex from, Vertex to, const Matrix &T_from_to, const CovMatrix & COV_from_to)
{
  { // Add to buffer
    std::unique_lock<std::mutex> lock(new_data_mutex_);
    new_data_buffer_.push_back(std::make_tuple(from, to, T_from_to, COV_from_to));
  }
  // notify main thread
  new_data_cond_var_.notify_one();
}

template<typename T>
void OptimizerMT<T>::Run()
{
  std::cout << "[OptimizerMT] Starting main thread...\n";
  stop_ = false;
  main_thread_ = std::thread(&OptimizerMT<T>::Main, this);
}

template<typename T>
void OptimizerMT<T>::Main()
{
  // main loop
  while(not stop_) {

    // Try to get new data, waits if no data
    {
      std::unique_lock<std::mutex> lock(new_data_mutex_);
      if (new_data_buffer_.empty())
        new_data_cond_var_.wait(lock, [this] {
          return (not this->new_data_buffer_.empty()) or this->stop_;
        });
      // Check for shutdown
      if (stop_) break;
      // Fetch all data in the input buffer
      this->data_buffer_.clear();
      std::copy(new_data_buffer_.begin(), new_data_buffer_.end(), std::back_inserter(this->data_buffer_));
      new_data_buffer_.clear();
    }

    this->ProcessData();

  } // end main loop
}

template<typename T>
void OptimizerMT<T>::PrepareForOptimization()
{
   auto graph_lock = map_manager_mt_ptr_->GetGraphLock();
   Base::PrepareForOptimization();
}

template<typename T>
void OptimizerMT<T>::UpdateAfterOptimization()
{
  auto graph_lock = map_manager_mt_ptr_->GetGraphLock();
  Base::UpdateAfterOptimization();
}

} // pgslam

#endif // PGSLAM_OPTIMIZER_MT_HPP
