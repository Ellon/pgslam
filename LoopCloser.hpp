#ifndef PGSLAM_LOOP_CLOSER_HPP
#define PGSLAM_LOOP_CLOSER_HPP

#include "LoopCloser.h"

#include <iostream>

namespace pgslam {

template<typename T>
LoopCloser<T>::LoopCloser(MapManagerPtr map_manager_ptr) :
  stop_{false},
  map_manager_ptr_{map_manager_ptr}
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

    }

}

}

} // pgslam

#endif // PGSLAM_LOOP_CLOSER_HPP


