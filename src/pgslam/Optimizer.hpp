#ifndef PGSLAM_OPTIMIZER_HPP
#define PGSLAM_OPTIMIZER_HPP

#include "Optimizer.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>


namespace pgslam {

template<typename T>
Optimizer<T>::Optimizer(MapManagerPtr map_manager_ptr) :
  map_manager_ptr_{map_manager_ptr}
{}

template<typename T>
Optimizer<T>::~Optimizer()
{}

template<typename T>
void Optimizer<T>::AddNewData(Vertex from, Vertex to, const Matrix &T_from_to, const CovMatrix & COV_from_to)
{
  data_buffer_.clear();
  data_buffer_.push_back(std::make_tuple(from, to, T_from_to, COV_from_to));
  ProcessData();
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
  return gtsam::Pose3(gtsam::Rot3(mat.block(0,0,3,3).template cast<double>()), gtsam::Point3(mat.col(3).head(3).template cast<double>()));
}

template<typename T>
typename Optimizer<T>::Matrix Optimizer<T>::GtsamPoseToPmPose(const gtsam::Pose3 &pose)
{
  return pose.matrix().cast<T>();
}

template<typename T>
void Optimizer<T>::ProcessData()
{
  std::cout << "[Optimizer] Building factor graph with "
          << data_buffer_.size()
          << " new loop closing factors\n";

  PrepareForOptimization();

  std::cout << "[Optimizer] Optimizing using the factor graph\n";

  // Optimize
  current_estimate_ = gtsam::LevenbergMarquardtOptimizer(factor_graph_, initial_values_).optimize();

  std::cout << "[Optimizer] Updating graph poses and adding loop closing edges\n";

  UpdateAfterOptimization();

  std::cout << "[Optimizer] Finished\n";

}

template<typename T>
void Optimizer<T>::PrepareForOptimization()
{
  const auto & graph = map_manager_ptr_->GetGraph();

  // Clear variables before repopulating them
  factor_graph_ = gtsam::NonlinearFactorGraph();
  initial_values_.clear();

  // Populate factor graph from edge data
  const auto es = boost::edges(graph);
  std::for_each(es.first, es.second, [this, &graph](auto e) {
    auto from = graph[boost::source(e, graph)].id;
    auto to = graph[boost::target(e, graph)].id;
    gtsam::Pose3 mean(this->PmPoseToGtsamPose(graph[e].T_from_to));
    gtsam::noiseModel::Gaussian::shared_ptr noise = gtsam::noiseModel::Gaussian::Covariance(this->PmCovToGtsamCov(graph[e].cov_from_to).template cast<double>());

    this->factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(from, to, mean, noise));
  });

  // Populate factor graph with new loop closing edges
  std::for_each(data_buffer_.begin(), data_buffer_.end(), [this, &graph](auto & data) {
    Vertex from, to;
    Matrix T_from_to;
    CovMatrix COV_from_to;
    std::tie(from, to, T_from_to, COV_from_to) = data;

    auto id_from = graph[from].id;
    auto id_to = graph[to].id;
    gtsam::Pose3 mean(this->PmPoseToGtsamPose(T_from_to));
    gtsam::noiseModel::Gaussian::shared_ptr noise = gtsam::noiseModel::Gaussian::Covariance(this->PmCovToGtsamCov(COV_from_to).template cast<double>());

    this->factor_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(id_from, id_to, mean, noise));
  });

  // Populate initial values from vertices data
  const auto vs = boost::vertices(graph);
  std::for_each(vs.first, vs.second, [this, &graph](auto v) {
    auto id = graph[v].id;
    gtsam::Pose3 value(this->PmPoseToGtsamPose(graph[v].optimized_T_world_kf));

    this->initial_values_.insert(id, value);
  });

  // Fix one of the vertices
  auto fixed_vertex = map_manager_ptr_->GetFixedVertex();
  auto id = graph[fixed_vertex].id;
  gtsam::Pose3 prior_mean(PmPoseToGtsamPose(graph[fixed_vertex].optimized_T_world_kf));
  using Vector6 = Eigen::Matrix<T,6,1>;
  auto sigmas = (Vector6() << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished();
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas.template cast<double>());
  factor_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(id, prior_mean, prior_noise));

}

template<typename T>
void Optimizer<T>::UpdateAfterOptimization()
{
  const auto & graph = map_manager_ptr_->GetGraph();

  // Update the graph with the current estimate
  const auto vs = boost::vertices(graph);
  std::for_each(vs.first, vs.second, [this, &graph](auto v) {
    this->map_manager_ptr_->UpdateKeyframeTransform(v, this->GtsamPoseToPmPose(this->current_estimate_.template at<gtsam::Pose3>(graph[v].id)));
  });

  // Add loop closing edges to the graph
  std::for_each(data_buffer_.begin(), data_buffer_.end(), [this](auto & data) {
    Vertex from, to;
    Matrix T_from_to;
    CovMatrix COV_from_to;
    std::tie(from, to, T_from_to, COV_from_to) = data;
    this->map_manager_ptr_->AddLoopClosingConstraint(from, to, T_from_to, COV_from_to);
  });

}

} // pgslam

#endif // PGSLAM_OPTIMIZER_HPP
