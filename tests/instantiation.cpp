#include "pgslam/PoseGraphSlam.h"
#include "pgslam/PoseGraphSlamMT.h"

int main() {
  {
    pgslam::PoseGraphSlam<float> slam;
  }

  {
    pgslam::PoseGraphSlam<double> slam;
  }

  {
    pgslam::PoseGraphSlamMT<float> slam;
  }

  {
    pgslam::PoseGraphSlamMT<double> slam;
  }
}