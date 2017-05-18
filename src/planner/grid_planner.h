#pragma once
#include "nav_global/defs.h"
#include "nav_global/distance_map.h"
#include <limits>
#define DEBUG 0
//Debug

#if DEBUG==1
#define PRINT_DEBUG( x ) std::cout << x << endl
#else
#define PRINT_DEBUG( x )
#endif
namespace thin_navigation {


/*!
     Simple planner algorithm that implements an A* planner based on a Dijkstra heuristic markov localization algorithm on a 2D gridmap.
     The map is represented through an 8 bit grayscale image, and a resolution meters/pixel.
     The plan is a list of cells to be traversed by the robot

     TODO: Cristiano, please provide a simple code snipped with a use case;
  */
struct MyQueue: public std::multimap<float, DistanceMapCell*> {
    void push(DistanceMapCell* c){
        insert(std::make_pair(c->distance, c));
    }
    void push(float f,DistanceMapCell* c){
        insert(std::make_pair(f, c));
    }
    inline float topDistance() {
        if(empty())
            return -1;
        return begin()->first;
    }
    DistanceMapCell* top(){
        if (empty()){
            return NULL;
        }else{
            return begin()->second;
        }
    }
    void pop(){
        if (!empty())
            erase(begin());
    }
};

class GridPlanner {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    GridPlanner();

    //! call this to load the map in the planner
    //! @param m: an 8 bit grayscale image
    //! @param resolution: size in meters of a map pixel
    //! @param occ_threshold: values of a pixel values below this are considered occupies
    //! @param free_threshold: values of a pixel above this are free space
    void setMap(const UnsignedCharImage& m,
                float resolution,
                unsigned char occ_threshold,
                unsigned char free_threshold);

    //! initializes the planner (after setting a map)
    //! expands the map at the distance _distance_threshold to compute the cost of the cells
    //! grows the obstacles of robot radius
    virtual void init();


    //! sets/gets the robot radius in meters, The planner will ensure no path is closer
    //! to an obstacle than this threshold
    inline float robotRadius() const {return _robot_radius;}
    inline void setRobotRadius(float r) {_robot_radius=r;}

    //!set the dimension in meters of the local map
    inline void setLocalMapDimension(float td) {_local_map_dimension=td;}

    //! sets the goal of the planner
    //! causes the heuristic to be recomputed on all the map
    //! if false the location is invalid
    inline bool setGoal(const Eigen::Vector3f g) {
        _goal=g;
        _goal_rc=world2grid(g);
        _heuristic_good=computeHeuristic();
        _plan_found=""; // the plan requires to set the robot pose;
        return _heuristic_good;
    }

    //! sets the robot position
    inline void setRobotPose(const Eigen::Vector3f pose) { _robot_pose=pose; _robot_pose_rc=world2grid(pose);}

    //! starts the planning(the goal must be set)
    //! returns true on success
    std::string computePlan();

    //! returns the plan of the robot as a sequence of x,y,theta tuples
    inline const Vector4fVector& plan() const { return _local_plan;}

    //! integrates an observation in the distance map, and recomputes the map in the neighborhood of the robot
    void updateTemporaryMap(const Vector2fVector& observation);


    //! paints the state in a rgb image (trajectory)
    void paintState(RGBImage& img, bool use_distance_map=false);


    //! sets the coefficient of the obstacle distance
    inline void setDistanceCostFactor(float dc) {_distance_cost_factor=dc;}

    //! gets the coefficient of the obstacle distance
    inline float distanceCostFactor() const {return _distance_cost_factor;}

    inline void setGoalToleranceT(float goal_tolerance_t_){_goal_tollerance_t = goal_tolerance_t_;}
    inline void setGoalToleranceR(float goal_tolerance_r_){_goal_tollerance_r = goal_tolerance_r_;}

    //! true if the plan is valid;
    inline bool isPlanFound() const {return (_plan_found!="GO" && _plan_found!="NEAR")?false:true;}
    
    //! sets/gets the distance threshold used to expand the obstacles
    inline float distanceThreshold() const {return _distance_threshold;}
    inline void setDistanceThreshold(float dt) {_distance_threshold=dt;}

    inline UnsignedCharImage getTempMap() const {return _local.map_image;}

    inline void setCompute_global_path(bool compute_global_path){_compute_global_path=compute_global_path; }

    inline float persistency(){return _persistency;}
    inline void setPersistency(float persistency_){_persistency = persistency_;}

protected:
    struct MyMap{
        UnsignedCharImage map_image;
        float resolution, inverse_resolution;
        float width,height;
        int rows,cols;
        IntImage int_map;
        IntImage assoc_map;
        DistanceMap distance_map;
        DistanceMap heuristic;
        FloatImage distances;
        FloatImage cost;
    };


    inline float cost( float d ) const {
        float c=_distance_cost_factor/(fabs(d)+0.001);
        if (d<_robot_radius){
            c+=_bump_cost;
        }else if (d<2*_robot_radius){
            c+=(_bump_cost/3);
        }
        return c;
    }

    inline Eigen::Vector2i world2grid(const Eigen::Vector2f p) {
        return Eigen::Vector2i(p.x()*_global.inverse_resolution, p.y()*_global.inverse_resolution);
    }
    inline Eigen::Vector2i world2grid(const Eigen::Vector3f p) {
        return Eigen::Vector2i(p.x()*_global.inverse_resolution, p.y()*_global.inverse_resolution);
    }

    inline Eigen::Vector2f grid2world(const Eigen::Vector2i p) {
        return Eigen::Vector2f(p.x()*_global.resolution, p.y()*_global.resolution);
    }
    inline void polar(float x, float y, float& r, float& theta)
    {
        r = sqrt((pow(x,2))+(pow(y,2)));
        theta = atan2(y,x);

    }

    //! recomputes the heuristic from the goal to all cells
    //! returns false on fail
    bool computeHeuristic();
    
    //! computes the plan using astar on the heuristic given
    //! returns false on fail
    std::string planning();

    void computeEndPoints(const Vector2fVector& observation);

    bool search(const FloatImage& map, DistanceMap& heuristic, const DistanceMapCell& goal, Vector4fVector &plan);

    void reconstructPath(DistanceMapCell* start,  Vector4fVector& plan,FloatImage map, Eigen::Vector2i offset);
    void reconstructInversePath(DistanceMapCell *start, Vector4fVector &plan, FloatImage map);
    // global params
    Eigen::Vector3f _robot_pose;
    Eigen::Vector2i _robot_pose_rc;
    Eigen::Vector3f _old_robot_pose;
    Eigen::Vector3f _goal;
    Eigen::Vector2i _goal_rc;
    Eigen::Vector2i _local_offset;
    Eigen::Vector3f _local_pose;
    Eigen::Vector2i _local_pose_rc;

    bool _heuristic_good; // toggled the heuristic is valid
    std::string _plan_found;   // true when a plan is found;
    float _distance_cost_factor; // cost of a cell
    float _distance_threshold; // distance used to expand the obstacle map
    float _local_map_dimension;
    float _persistency;
    float _goal_tollerance_t; //tollerance for reach the goal
    float _goal_tollerance_r;
    float _thresh_same_point;
    float _heuristic_weight;
    float _nearest_dynamic_object;
    //robot configuration
    float _robot_radius;
    bool _consider_unknown_as_obstacle;
    bool _compute_global_path;
    float _bump_cost;
    float _actual_tv,_actual_rv;
    Eigen::Vector3f _next_point;
    //last laser valid endpoints

    MyMap _local,_global,_dynamic;
     DistanceMap search_map;
     FloatImage _temp_distances;
    Vector3fVector _prec_endpoints;
    std::vector<float> _endpoint_distances;

    //! variable where the plan is stored
    Vector4fVector _local_plan;
    Vector4fVector _global_plan;
    Vector2fVector _last_endpoints;




};

}
