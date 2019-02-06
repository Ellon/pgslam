#ifndef PGSLAM_MAP_MANAGER_MT_HPP
#define PGSLAM_MAP_MANAGER_MT_HPP

#include "MapManagerMT.h"

namespace pgslam {

template<typename T>
MapManagerMT<T>::MapManagerMT()
{}

template<typename T>
MapManagerMT<T>::~MapManagerMT()
{}

template<typename T>
std::unique_lock<std::mutex> MapManagerMT<T>::GetGraphLock()
{
  return std::unique_lock<std::mutex>(graph_mutex_);
}

} // pgslam

#endif // PGSLAM_MAP_MANAGER_MT_HPP
