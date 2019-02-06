#ifndef PGSLAM_LOOP_CLOSER_MT_H
#define PGSLAM_LOOP_CLOSER_MT_H

#include "types.h"
#include "MapManagerMT.h"
#include "OptimizerMT.h"

#include "LoopCloser.h"

namespace pgslam {

template<typename T>
class LoopCloserMT : public LoopCloser<T>{
public:
  using Base = LoopCloser<T>;
  using Ptr = std::shared_ptr<LoopCloserMT<T>>;

  using MapManagerMTPtr = typename MapManagerMT<T>::Ptr;
  using OptimizerMTPtr = typename OptimizerMT<T>::Ptr;

  IMPORT_PGSLAM_TYPES(T)

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

  //! Object to store shared data (graph of keyframes)
  MapManagerMTPtr map_manager_mt_ptr_;

public:
  LoopCloserMT(MapManagerMTPtr map_manager_mt_ptr, OptimizerMTPtr optimizer_mt_ptr);
  virtual ~LoopCloserMT();

  void Run();
  void Main();

  virtual void AddNewVertex(Vertex v);
  virtual bool ProcessLocalMapCandidate();

};

} // pgslam

#include "LoopCloserMT.hpp"

#endif // PGSLAM_LOOP_CLOSER_MT_H
