#include "ros_utils.h"

namespace thin_navigation{
  using namespace std;


  //! returns the time in milliseconds
  double getTime() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return 1e3*tv.tv_sec+1e-3*tv.tv_usec;
  }

  Eigen::Vector3f convertPose(const tf::StampedTransform& t) {
    double yaw,pitch,roll;
    tf::Matrix3x3 mat =  t.getBasis();
    mat.getRPY(roll, pitch, yaw);
    return Eigen::Vector3f(t.getOrigin().x(), t.getOrigin().y(), yaw);
  }

  double getYaw(tf::Pose& t) {
    double yaw, pitch, roll;
    t.getBasis().getEulerYPR(yaw,pitch,roll);
    return yaw;
  }
}
