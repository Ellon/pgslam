#ifndef PGSLAM_MAP_MANAGER_MT_H
#define PGSLAM_MAP_MANAGER_MT_H

#include <mutex>

#include "MapManager.h"

namespace pgslam {

template<typename T>
class MapManagerMT : public MapManager<T> {
public:
  using Base = MapManager<T>;
  using Ptr = std::shared_ptr<MapManagerMT<T>>;

private:
  //! Mutex that controls access to graph_
  std::mutex graph_mutex_;

public:
  MapManagerMT(/* args */);
  virtual ~MapManagerMT();

  std::unique_lock<std::mutex> GetGraphLock();

};

} // pgslam

#include "MapManagerMT.hpp"

#endif // PGSLAM_MAP_MANAGER_MT_H
