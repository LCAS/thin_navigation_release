#include "gradient_controller.h"
namespace thin_navigation {

GradientController::GradientController()
{
  Ktv_=1.0f;
  Krv_=2.0f;
  repulsive_r_scale_=.15;
  repulsive_t_scale_=.2;
  gradient_map_size_=40; 
  img_resolution_inv_=1/0.05;
  max_t_acc_=.05;
  max_r_acc_=.05;
  min_attractor_distance_=.4*.4;
  max_attractor_distance_=2.3*2.3;
  safety_threshold_=.3;
  robot_radius_=.25;
  max_tv_=1.0;
  max_rv_=.5;
}

void GradientController::showAttractivePoint(const FloatImage& dist_map, const Eigen::Vector3f& attractive_point)
{
  cv::Mat im;
  cv::cvtColor(dist_map,im,CV_GRAY2BGR);
  Eigen::Vector2f ap(attractive_point(0)*img_resolution_inv_+im.cols/2, im.rows-(attractive_point(1)*img_resolution_inv_+im.rows/2));
  cv::Scalar color(0,0,255);
  if(ap(0)>=0&&ap(0)<im.rows&&ap(1)>=0&&ap(1)<im.cols)
    cv::circle(im, cv::Point(ap(0),ap(1)), 3, color);
  cv::imshow("attractor",im);
  cv::waitKey(20);
}

bool GradientController::isSafePath(const FloatImage& dist_map, const Eigen::Vector3f& attractive_point)
{
  cv::Point ap(attractive_point(0)*img_resolution_inv_+dist_map.cols/2, 
               dist_map.rows-(attractive_point(1)*img_resolution_inv_+dist_map.rows/2));
  cv::LineIterator lit(dist_map, cv::Point(dist_map.cols/2, dist_map.rows/2), ap, 4);
  
  for(int i = 0; i < lit.count; i++, ++lit)
    if(*(const float*)*lit<(safety_threshold_+robot_radius_))
    {
      return false;
    }
  return true;
}

void GradientController::computeAttractivePoint(const Vector4fVector& plan, const FloatImage& dist_map, const Eigen::Vector3f& robot_pose, Eigen::Vector3f& attractive_point)
{
  cv::Mat dist_map_rotated;
  cv::Mat R=cv::getRotationMatrix2D(cv::Point2f((float)dist_map.rows/2,(float)dist_map.cols/2), -( (float)robot_pose(2)*180/M_PI - 90), 1);
  cv::warpAffine(dist_map, dist_map_rotated, R, dist_map.size());
  Eigen::Isometry2f robot_inverse_transform=v2t(robot_pose).inverse();
  
  int i=plan.size()-1;
  int j=1;
  int min_dist_idx=0;
  
  Eigen::Isometry2f plan_transform=v2t(Eigen::Vector3f(plan[i].x(),plan[i].y(),plan[i].z()));
  attractive_point=t2v(robot_inverse_transform*plan_transform);
  i--;
  while(i>=min_dist_idx)
  {
    
    if(min_dist_idx==0)
    {
      plan_transform=v2t(Eigen::Vector3f(plan[j].x(),plan[j].y(),plan[j].z()));
      Eigen::Vector3f p=t2v(robot_inverse_transform*plan_transform);
      if(p.head(2).squaredNorm()>=min_attractor_distance_)
      {
        attractive_point=p;
        min_dist_idx=j;
      }
      else
      {
        j++; 
      }
    }
    
    plan_transform=v2t(Eigen::Vector3f(plan[i].x(),plan[i].y(),plan[i].z()));
    Eigen::Vector3f attr_p=t2v(robot_inverse_transform*plan_transform);
    if(attr_p.head(2).squaredNorm()>max_attractor_distance_)
    {
      i--;
      continue;
    }
    if(i==min_dist_idx||isSafePath(dist_map_rotated,attr_p))
    {
      attractive_point=attr_p;
      break;
    }
    i--;
  }
  //DEBUG
  //showAttractivePoint(dist_map_rotated, attractive_point);
}

void GradientController::computeRepulsiveForce(const FloatImage& dist_map, const Eigen::Vector3f& robot_pose, Eigen::Vector2f& force_vector)
{
  cv::Mat gradient_x, gradient_y;
  int scale = 0.05*img_resolution_inv_;
  int delta = 0;
  int ddepth = CV_32FC1;

  int gradient_size=gradient_map_size_;
  
  cv::Mat dist_map_small=dist_map(cv::Range(dist_map.rows/2-gradient_size/2,dist_map.rows/2+gradient_size/2),
                                  cv::Range(dist_map.cols/2-gradient_size/2,dist_map.cols/2+gradient_size/2)).clone();
  
  //TODO Avoid warpAffine for improving performance
  cv::Mat R=cv::getRotationMatrix2D(cv::Point2f((float)dist_map_small.rows/2,(float)dist_map_small.cols/2), -( (float)robot_pose(2)*180/M_PI - 90), 1);
  cv::warpAffine(dist_map_small, dist_map_small, R, dist_map_small.size());

  /// Gradient X
  gradient_x.create(dist_map_small.size(),dist_map_small.type());
  cv::Scharr( dist_map_small, gradient_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT );

  /// Gradient Y
  gradient_y.create(dist_map_small.size(),dist_map_small.type());
  cv::Scharr( dist_map_small, gradient_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT );

  
  force_vector=(1/dist_map_small.at<float>(dist_map_small.rows/2, dist_map_small.cols/2))*Eigen::Vector2f(gradient_x.at<float>(gradient_x.rows/2, gradient_x.cols/2),
                                                                                                      gradient_y.at<float>(gradient_y.rows/2, gradient_y.cols/2));
  
  //DEBUG
//   cv::resize(dist_map_small, dist_map_small_dbg, cv::Size(400,400));
//   cv::cvtColor(dist_map_small_dbg,dist_map_small_dbg, CV_GRAY2RGB);
//   cv::line(dist_map_small_dbg, cv::Point(dist_map_small_dbg.cols/2, dist_map_small_dbg.rows/2), cv::Point(dist_map_small_dbg.rows/2+(int)(force_vector(0)*200), dist_map_small_dbg.rows/2+(int)(force_vector(1)*200)),CV_RGB(255,0,0) );
//   
}

void GradientController::smoothVelocity(const Eigen::Vector2f& old_velocity,const Eigen::Vector2f& velocity, Eigen::Vector2f& smoothd_velocity)
{
  if(fabs(velocity(0)-old_velocity(0))<max_t_acc_)
    smoothd_velocity(0)=velocity(0);
  else if(velocity(0)>old_velocity(0))
    smoothd_velocity(0)=old_velocity(0)+max_t_acc_;
  else 
    smoothd_velocity(0)=old_velocity(0)-max_t_acc_;

  if(fabs(velocity(1)-old_velocity(1))<max_r_acc_)
    smoothd_velocity(1)=velocity(1);
  else if(velocity(1)>old_velocity(1))
    smoothd_velocity(1)=old_velocity(1)+max_r_acc_;
  else 
    smoothd_velocity(1)=old_velocity(1)-max_r_acc_;
}

void GradientController::computeDesiredVelocity(const Eigen::Vector3f& attractive_point, Eigen::Vector2f& desired_velocity)
{
  //std::cerr << "GradientController::computeDesiredVelocity: " << Ktv_ << " " << Krv_ << std::endl;

  //TODO implement a better solution
  desired_velocity(0)=Ktv_*attractive_point(0);
  float angle_diff=(float)atan2(attractive_point(1), attractive_point(0));
  desired_velocity(1)=Krv_*angle_diff;

//   std::cout<<"attr_point:"<<attractive_point.transpose()<<" angle_diff "<<angle_diff<<std::endl;
}

void GradientController::computeVelocity(const FloatImage& dist_map, const FloatImage& dyn_dist_map, const Eigen::Vector3f &robot_pose, const Eigen::Vector2f& desired_velocity, Eigen::Vector2f& velocity)
{
  
  if(fabs(desired_velocity(0))<.03)
  {
    velocity=desired_velocity;
    return;
  }
  
  Eigen::Vector2f force_vector,dyn_force_vector;
  computeRepulsiveForce(dist_map, robot_pose, force_vector);
  computeRepulsiveForce(dyn_dist_map,robot_pose,dyn_force_vector);
  velocity=desired_velocity;
  velocity(0)+=(repulsive_t_scale_*dyn_force_vector(0) + (repulsive_t_scale_/4)*force_vector(0));
  float angle=(float)atan2(dyn_force_vector(1),-dyn_force_vector(0));
  velocity(1)-=repulsive_r_scale_*angle;

  angle=(float)atan2(force_vector(1),-force_vector(0));
  velocity(1)-=(repulsive_r_scale_/4)*angle;

  //
  if(desired_velocity(0)*velocity(0)<0) velocity(0)=0;
  else if(fabs(desired_velocity(0))<fabs(velocity(0))) velocity(0)=desired_velocity(0);
  
  //for fixed dyn obstacles
  float min_tv=0.03;
  if(fabs(velocity(0))<min_tv)
    (fabs(desired_velocity(0))>fabs(velocity(0)))? velocity(0)=min_tv:velocity(0)=desired_velocity(0);
}

void GradientController::computeVelocity(const FloatImage& dist_map,const FloatImage& dyn_dist_map, const Eigen::Vector3f &robot_pose, const Eigen::Vector3f& attractive_point, Eigen::Vector2f& velocity)
{
  Eigen::Vector2f desired_velocity;
  computeDesiredVelocity(attractive_point, desired_velocity);

  computeVelocity(dist_map,dyn_dist_map, robot_pose, desired_velocity, velocity);
}

void GradientController::computeVelocity(const FloatImage& dist_map, const FloatImage& dyn_dist_map, const Eigen::Vector3f& robot_pose, const Vector4fVector& plan, Eigen::Vector2f& velocity)
{
  Eigen::Vector3f attractive_point;
  computeAttractivePoint(plan, dist_map, robot_pose, attractive_point);
  computeVelocity(dist_map,dyn_dist_map, robot_pose, attractive_point, velocity);

  if(velocity(0)<0) 
    velocity(0)=0;
  if(velocity(0)>max_tv_)
    velocity(0)=max_tv_;
  if(fabs(velocity(1))>max_rv_)
  {
    if(velocity(1)>0)
      velocity(1)=max_rv_;
    if(velocity(1)<0)
      velocity(1)=-max_rv_;
  }
}


}
