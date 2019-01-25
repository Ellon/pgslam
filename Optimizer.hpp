#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

namespace pgslam {

template<typename T>
Optimizer<T>::Optimizer(MapManagerPtr map_manager_ptr) :
  stop_{false},
  map_manager_ptr_{map_manager_ptr}
{}

template<typename T>
Optimizer<T>::~Optimizer()
{
  stop_ = true;
  // Threads may be waiting, notify all.
  new_data_cond_var_.notify_all();
  if (main_thread_.joinable())
    main_thread_.join();
}

template<typename T>
void Optimizer<T>::AddNewData(Vertex from, Vertex to, const Matrix &T_from_to, const CovMatrix & COV_from_to)
{
  { // Add to buffer
    std::unique_lock<std::mutex> lock(new_data_mutex_);
    new_data_buffer_.push_back(std::make_tuple(from, to, T_from_to, COV_from_to));
  }
  // notify main thread
  new_data_cond_var_.notify_one();
}

template<typename T>
void Optimizer<T>::Run()
{
  std::cout << "[Optimizer] Starting main thread...\n";
  stop_ = false;
  main_thread_ = std::thread(&Optimizer<T>::Main, this);
}

template<typename T>
void Optimizer<T>::Main()
{
  // main loop
  while(not stop_) {

    // Try to get new data, waits if no data
    DataBuffer data_buffer;
    {
      std::unique_lock<std::mutex> lock(new_data_mutex_);
      if (new_data_buffer_.empty())
        new_data_cond_var_.wait(lock, [this] {
          return (not this->new_data_buffer_.empty()) or this->stop_;
        });
      // Check for shutdown
      if (stop_) break;
      // Fetch all data in the input buffer
      std::copy(new_data_buffer_.begin(), new_data_buffer_.end(), std::back_inserter(data_buffer));
      new_data_buffer_.clear();
    }

    std::cout << "[Optimizer] Optimizing graph with "
              << data_buffer.size()
              << " new loop closing edges\n";

  }
}


} // pgslam

#endif // OPTIMIZER_HPP
