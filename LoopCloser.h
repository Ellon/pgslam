#ifndef PGSLAM_LOOP_CLOSER_H
#define PGSLAM_LOOP_CLOSER_H

#include <deque>
#include <thread>

#include "types.h"
#include "MapManager.h"

namespace pgslam {

template<typename T>
class LoopCloser {
public:
  using Ptr = std::shared_ptr<LoopCloser<T>>;

  using MapManagerPtr = typename MapManager<T>::Ptr;

  IMPORT_PGSLAM_TYPES(T)

public:
  LoopCloser(MapManagerPtr map_manager_ptr);
  ~LoopCloser();

  void AddNewVertex(Vertex v);
  void Run();
  void Main();


private:
  // Variables used to input data in the thread
  //! Variable used to stop the thread
  bool stop_ = {false};
  //! Buffer with new data to be processed
  std::deque<Vertex> new_vertex_buffer_;
  //! Mutex to control access to new_vertex_buffer_
  std::mutex new_vertex_mutex_;
  //! Condition variable to inform localization thread of new data
  std::condition_variable new_vertex_cond_var_;
  //! Main thread object
  std::thread main_thread_;

  //! Object to store shared data
  MapManagerPtr map_manager_ptr_;

};

} // pgslam

#include "LoopCloser.hpp"

#endif // PGSLAM_LOOP_CLOSER_H
