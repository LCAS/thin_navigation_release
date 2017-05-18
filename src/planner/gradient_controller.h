#include "nav_global/defs.h"

namespace thin_navigation {


class GradientController {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  GradientController();
  
  void computeVelocity(const FloatImage& dist_map, const FloatImage& dyn_dist_map,const Eigen::Vector3f& robot_pose,
                       const  Eigen::Vector2f& desired_velocity, Eigen::Vector2f& velocity);
  void computeVelocity(const FloatImage& dist_map, const FloatImage& dyn_dist_map, const Eigen::Vector3f& robot_pose,
                       const Eigen::Vector3f& attractive_point, Eigen::Vector2f& velocity);
  void computeVelocity(const FloatImage& dist_map, const FloatImage& dyn_dist_map, 
                       const Eigen::Vector3f& robot_pose,
                       const Vector4fVector& plan, Eigen::Vector2f& velocity);
  void smoothVelocity(const Eigen::Vector2f& old_velocity, const Eigen::Vector2f& velocity, Eigen::Vector2f& smoothd_velocity);
  
  inline void setAttractionParameters(const float& Ktv, const float& Krv){ Ktv_=Ktv; Krv_=Krv; }
  inline void setRepulsionParameters(const float& t_scale, const float& r_scale){ repulsive_t_scale_=t_scale; repulsive_r_scale_=r_scale; }
  inline void setImageResolution(const float& resolution){img_resolution_inv_=1/resolution;}
  inline void setMaxVelocity(const float& max_tv, const float& max_rv){max_tv_=max_tv; max_rv_=max_rv;}
  inline void setMaxAccelerations(const float& t_acc, const float& r_acc){ max_t_acc_=t_acc; max_r_acc_=r_acc;}
  inline void setAttractorDistances(const float& min_dist, const float& max_dist){min_attractor_distance_=min_dist; max_attractor_distance_=max_dist;}
  inline void setRobotRadius(const float& radius){robot_radius_=radius;}
  inline void setSafePathThreshold(const float& th){safety_threshold_=th;}

protected:
  
  void computeRepulsiveForce(const FloatImage& dist_map, const Eigen::Vector3f& robot_pose, Eigen::Vector2f& force_vector);
  void computeDesiredVelocity( const Eigen::Vector3f& attractive_point, Eigen::Vector2f& desired_velocity);
  void computeAttractivePoint(const Vector4fVector& plan, const FloatImage& dist_map, const Eigen::Vector3f& robot_pose, Eigen::Vector3f& attractive_point);
  bool isSafePath(const FloatImage& dist_map, const Eigen::Vector3f& attractive_point);
  
  void showAttractivePoint(const FloatImage& dist_map, const Eigen::Vector3f& attractive_point);
  
  float Ktv_, Krv_;
  float repulsive_t_scale_, repulsive_r_scale_;
  int gradient_map_size_;
  float img_resolution_inv_;
  
  float max_tv_, max_rv_;
  float max_t_acc_, max_r_acc_;
  float min_attractor_distance_, max_attractor_distance_;
  float robot_radius_;
  
  //distance from obstacles (in meters) 
  float safety_threshold_;

};

}
