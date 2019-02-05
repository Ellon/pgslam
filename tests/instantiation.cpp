#include "pgslam/PoseGraphSlam.h"

int main() {
  {
    pgslam::PoseGraphSlam<float> slam;
  }

  {
    pgslam::PoseGraphSlam<double> slam;
  }
}