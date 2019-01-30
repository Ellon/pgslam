#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

#include "Optimizer.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>


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

//! Swap submatrices to convert covariance from PM order [x,y,z,rx,ry,rz] to gtsam's order [rx,ry,rz,x,y,z]
template<typename T>
typename Optimizer<T>::Matrix Optimizer<T>::PmCovToGtsamCov(const Matrix &mat)
{
  Matrix cov(6, 6);
  cov.block(0, 0, 3, 3) = mat.block(3, 3, 3, 3);
  cov.block(3, 3, 3, 3) = mat.block(0, 0, 3, 3);
  cov.block(3, 0, 3, 3) = mat.block(0, 3, 3, 3);
  cov.block(0, 3, 3, 3) = mat.block(3, 0, 3, 3);
  return std::move(cov);
}

template<typename T>
gtsam::Pose3 Optimizer<T>::PmPoseToGtsamPose(const Matrix &mat)
{
  return gtsam::Pose3(gtsam::Rot3(mat.block(0,0,3,3)), gtsam::Point3(mat.col(3).head(3)));
}

template<typename T>
typename Optimizer<T>::Matrix Optimizer<T>::GtsamPoseToPmPose(const gtsam::Pose3 &pose)
{
  return pose.matrix().cast<T>();
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

    gtsam::NonlinearFactorGraph factor_graph;
    gtsam::Values initial_values;
    {
      auto graph_lock = map_manager_ptr_->GetGraphLock();
      const auto & graph = map_manager_ptr_->GetGraph();

      // Populate factor graph from edge data
      const auto es = boost::edges(graph);
      std::for_each(es.first, es.second, [this, &graph, &factor_graph](auto e) {
        auto from = graph[boost::source(e, graph)].id;
        auto to = graph[boost::target(e, graph)].id;
        gtsam::Pose3 mean(this->PmPoseToGtsamPose(graph[e].T_from_to));
        gtsam::noiseModel::Gaussian::shared_ptr noise = gtsam::noiseModel::Gaussian::Covariance(this->PmCovToGtsamCov(graph[e].cov_from_to));

        factor_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(from, to, mean, noise));
      });

      // Populate factor graph with new loop closing edges
      std::for_each(data_buffer.begin(), data_buffer.end(), [this, &factor_graph, &graph](auto & data) {
        Vertex from, to;
        Matrix T_from_to;
        CovMatrix COV_from_to;
        std::tie(from, to, T_from_to, COV_from_to) = data;

        auto id_from = graph[from].id;
        auto id_to = graph[to].id;
        gtsam::Pose3 mean(this->PmPoseToGtsamPose(T_from_to));
        gtsam::noiseModel::Gaussian::shared_ptr noise = gtsam::noiseModel::Gaussian::Covariance(this->PmCovToGtsamCov(COV_from_to));

        factor_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(id_from, id_to, mean, noise));
      });

      // Populate initial values from vertices data
      const auto vs = boost::vertices(graph);
      std::for_each(vs.first, vs.second, [this, &graph, &initial_values](auto v) {
        auto id = graph[v].id;
        gtsam::Pose3 value(this->PmPoseToGtsamPose(graph[v].optimized_T_world_kf));

        initial_values.insert(id, value);
      });

      // Fix one of the vertices
      auto fixed_vertex = map_manager_ptr_->GetFixedVertex();
      auto id = graph[fixed_vertex].id;
      gtsam::Pose3 prior_mean(PmPoseToGtsamPose(graph[fixed_vertex].optimized_T_world_kf));
      using Vector6 = Eigen::Matrix<T,6,1>;
      auto sigmas = (Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished();
      gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
      factor_graph.add(gtsam::PriorFactor<gtsam::Pose3>(id, prior_mean, prior_noise));

    } // end of first graph lock scope

    // Optimize
    gtsam::Values current_estimate = gtsam::LevenbergMarquardtOptimizer(factor_graph, initial_values).optimize();

    {
      auto graph_lock = map_manager_ptr_->GetGraphLock();
      const auto & graph = map_manager_ptr_->GetGraph();

      // Update the graph with the current estimate
      const auto vs = boost::vertices(graph);
      std::for_each(vs.first, vs.second, [this, &graph, &current_estimate](auto v) {
        this->map_manager_ptr_->UpdateKeyframeTransform(v, this->GtsamPoseToPmPose(current_estimate.at<gtsam::Pose3>(graph[v].id)));
      });

      // Add loop closing edges to the graph
      std::for_each(data_buffer.begin(), data_buffer.end(), [this](auto & data) {
        Vertex from, to;
        Matrix T_from_to;
        CovMatrix COV_from_to;
        std::tie(from, to, T_from_to, COV_from_to) = data;
        this->map_manager_ptr_->AddLoopClosingConstraint(from, to, T_from_to, COV_from_to);
      });

    } // end of second graph lock scope

  }
}


} // pgslam

#endif // OPTIMIZER_HPP
