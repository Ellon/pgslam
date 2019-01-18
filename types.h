#ifndef TYPES_H
#define TYPES_H

#include <chrono>
#include <memory>

#include <pointmatcher/PointMatcher.h>
#include <boost/graph/adjacency_list.hpp>

namespace pgslam {

template<typename T>
struct Types
{
  // time
  using Time = std::chrono::time_point<std::chrono::high_resolution_clock>;

  // libpointmatcher
  using PM = PointMatcher<T>;
  using DP = typename PM::DataPoints;
  using DPPtr = std::shared_ptr<DP>;
  using Matrix = typename PM::Matrix;
  using CovMatrix = Eigen::Matrix<T, 6, 6 >;
  using ICPSequence = typename PM::ICPSequence;
  using TransformationPtr = std::shared_ptr<typename PM::Transformation>;
  using DataPointsFilters = typename PM::DataPointsFilters;
  using Label = typename DP::Label;
  using Labels = typename DP::Labels;

  // graph
  struct Keyframe {
    size_t id;
    DPPtr cloud_ptr;
    Matrix T_world_kf;
    Matrix optimized_T_world_kf;
    Time update_time;
  };

  struct Constraint {
    enum Type
    {
      kOdomConstraint,
      kLoopConstraint
    };
    //! Stores if the constraint comes from odometry or loop closing.
    Type type;
    //! The measurement
    Matrix T_from_to;
    //! The measurement error
    CovMatrix cov_from_to;

    //! The weight used when looking for local map compositions
    T weight;
  };

  using Graph = boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, Keyframe, Constraint>;
  using Vertex = typename boost::graph_traits<Graph>::vertex_descriptor;
  using Edge = typename boost::graph_traits<Graph>::edge_descriptor;

};

} // pgslam

#define IMPORT_PGSLAM_TYPES(TYPE)                                            \
  using Time = typename pgslam::Types<TYPE>::Time;                           \
  using PM = typename pgslam::Types<TYPE>::PM;                               \
  using DP = typename pgslam::Types<TYPE>::DP;                               \
  using DPPtr = typename pgslam::Types<TYPE>::DPPtr;                         \
  using Matrix = typename pgslam::Types<TYPE>::Matrix;                       \
  using CovMatrix = typename pgslam::Types<TYPE>::CovMatrix;                 \
  using ICPSequence = typename pgslam::Types<TYPE>::ICPSequence;             \
  using TransformationPtr = typename pgslam::Types<TYPE>::TransformationPtr; \
  using DataPointsFilters = typename pgslam::Types<TYPE>::DataPointsFilters; \
  using Keyframe = typename pgslam::Types<TYPE>::Keyframe;                   \
  using Label = typename pgslam::Types<TYPE>::Label;                         \
  using Labels = typename pgslam::Types<TYPE>::Labels;                       \
  using Constraint = typename pgslam::Types<TYPE>::Constraint;               \
  using Graph = typename pgslam::Types<TYPE>::Graph;                         \
  using Vertex = typename pgslam::Types<TYPE>::Vertex;                       \
  using Edge = typename pgslam::Types<TYPE>::Edge;

#endif // TYPES_H
