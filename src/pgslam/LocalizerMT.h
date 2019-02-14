#ifndef PGSLAM_LOCALIZER_MT_H
#define PGSLAM_LOCALIZER_MT_H

#include "types.h"
#include "MapManagerMT.h"

#include "Localizer.h"

namespace pgslam {

template<typename T>
class LocalizerMT : public Localizer<T> {
public:
  using Base = Localizer<T>;
  using Ptr = std::shared_ptr<LocalizerMT<T>>;

  using MapManagerMTPtr = typename MapManagerMT<T>::Ptr;

  IMPORT_PGSLAM_TYPES(T)

protected:
  // Variables used to input data in the thread
  //! Variable used to stop the thread
  bool stop_ = {false};
  //! Variable used to indicate if we are outdated wrt the graph
  bool outdated_ = {false};
  using InputData = std::tuple<unsigned long long int, std::string, Matrix, Matrix, DPPtr>;
  std::deque<InputData> new_data_buffer_; //!< Buffer with new data to be processed
  //! Mutex to control access to new_data_buffer_
  std::mutex new_data_mutex_;
  //! Condition variable to inform localization thread of new data
  std::condition_variable new_data_cond_var_;
  //! Main thread object
  std::thread main_thread_;

  //! Object to store shared data (graph of keyframes)
  MapManagerMTPtr map_manager_mt_ptr_;

public:
  LocalizerMT(MapManagerMTPtr map_manager_mt_ptr);
  virtual ~LocalizerMT();

  void Run();
  void Main();

  virtual void AddNewData(unsigned long long int timestamp,
                  std::string world_frame_id,
                  Matrix T_world_robot,
                  Matrix T_robot_sensor,
                  DPPtr cloud_ptr);

  virtual void UpdateFromGraph();

protected:
  virtual void ProcessFirstCloud(DPPtr cloud, const Matrix &T_world_robot);
  virtual void UpdateAfterIcp();

};

} // pgslam

#include "LocalizerMT.hpp"

#endif // PGSLAM_LOCALIZER_MT_H
