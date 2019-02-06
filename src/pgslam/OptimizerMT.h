#ifndef PGSLAM_OPTIMIZER_MT_H
#define PGSLAM_OPTIMIZER_MT_H

#include "types.h"
#include "MapManagerMT.h"

#include "Optimizer.h"

namespace pgslam {

template<typename T>
class OptimizerMT : public Optimizer<T> {
public:
  using Base = Optimizer<T>;
  using Ptr = std::shared_ptr<OptimizerMT<T>>;

  using MapManagerMTPtr = typename MapManagerMT<T>::Ptr;

  IMPORT_PGSLAM_TYPES(T)

  using DataBuffer = typename Base::DataBuffer;

private:
  // Variables used to input data in the thread
  //! Variable used to stop the thread
  bool stop_ = {false};
  DataBuffer new_data_buffer_; //!< Buffer with new data to be processed
  //! Mutex to control access to new_data_buffer_
  std::mutex new_data_mutex_;
  //! Condition variable to inform localization thread of new data
  std::condition_variable new_data_cond_var_;
  //! Main thread object
  std::thread main_thread_;

  //! Pointer to the object to store shared data
  MapManagerMTPtr map_manager_mt_ptr_;

public:
  OptimizerMT(MapManagerMTPtr map_manager_mt_ptr);
  virtual ~OptimizerMT();

  void Run();
  void Main();

  virtual void AddNewData(Vertex from, Vertex to, const Matrix &T_from_to, const CovMatrix & COV_from_to);
  virtual void PrepareForOptimization();
  virtual void UpdateAfterOptimization();
};

} // pgslam

#include "OptimizerMT.hpp"

#endif // PGSLAM_OPTIMIZER_MT_H
