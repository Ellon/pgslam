#ifndef PGSLAM_LOOP_CLOSER_MT_HPP
#define PGSLAM_LOOP_CLOSER_MT_HPP

#include "LoopCloserMT.h"

namespace pgslam {

template<typename T>
LoopCloserMT<T>::LoopCloserMT(MapManagerMTPtr map_manager_mt_ptr, OptimizerMTPtr optimizer_mt_ptr) :
  Base(map_manager_mt_ptr, optimizer_mt_ptr),
  stop_{false},
  map_manager_mt_ptr_{map_manager_mt_ptr}
{}

template<typename T>
LoopCloserMT<T>::~LoopCloserMT()
{
  stop_ = true;
  // Threads may be waiting, notify all.
  new_vertex_cond_var_.notify_all();
  if (main_thread_.joinable())
    main_thread_.join();
}

template<typename T>
void LoopCloserMT<T>::AddNewVertex(Vertex v)
{
  { // Add to buffer
    std::unique_lock<std::mutex> lock(new_vertex_mutex_);
    new_vertex_buffer_.push_back(v);
  }
  // notify main thread
  new_vertex_cond_var_.notify_one();
}

template<typename T>
void LoopCloserMT<T>::Run()
{
  std::cout << "[LoopCloserMT] Starting main thread...\n";
  stop_ = false;
  main_thread_ = std::thread(&LoopCloserMT<T>::Main, this);
}

template<typename T>
void LoopCloserMT<T>::Main()
{
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

    this->ProcessVertex(input_vertex);

  } // end main loop
}


template<typename T>
bool LoopCloserMT<T>::ProcessLocalMapCandidate()
{
  auto graph_lock = map_manager_mt_ptr_->GetGraphLock();
  return Base::ProcessLocalMapCandidate();
}

} // pgslam

#endif // PGSLAM_LOOP_CLOSER_MT_HPP
