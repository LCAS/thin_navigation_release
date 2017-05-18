#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/time.h>
#include <sstream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Twist.h>
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"

namespace thin_navigation {
  /**
     ! implements a simplistic control law that works well on differential drive robots
   */
  class ROSController {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    ROSController(ros::NodeHandle& nh, tf::TransformListener* listener=0);
    void subscribe(); // subscribe to all messages, and registers all publishers
  protected:
    float _trajectory_translational_threshold;  // maximum waypoint distance
    float _trajectory_rotational_threshold;     // maximum waypoint rotation
    float _min_curvature_radius;                // if the radius is below this value, pure rotation
    float _max_curvature_radius;                // if the radius is above this value, pure translation
    float _translational_gain;                  // controller gain for translation
    float _rotational_gain;                     // controller gain for rotation
    float _max_tv;                              // maximum translational velocity
    float _max_rv;                              // maximum rotational    velocity

    Eigen::Vector3fVector _path;                // path from the planner
    Eigen::Vector3f _robot_pose;                // robot pose from the localizer
    bool _is_running;                           //  if on executes the path
    ros::Publisher _cmd_vel_pub;
    ros::Subscriber _path_sub;
    tf::TransformListener* _transform_listener;
    ros::NodeHandle& _nh;
  };
}
