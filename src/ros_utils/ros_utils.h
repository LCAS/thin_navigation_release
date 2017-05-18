#pragma once

#include "ros/ros.h"
#include <sys/time.h>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"
#include "nav_global/defs.h"

namespace thin_navigation{

  //! returns the time in milliseconds
  double getTime() ;

  Eigen::Vector3f convertPose(const tf::StampedTransform& t) ;

  double getYaw(tf::Pose& t) ;
}
