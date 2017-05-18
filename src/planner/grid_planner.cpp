#include "grid_planner.h"
#include <cmath>
#include <iostream>
#include <sys/time.h>
#if defined(_WIN32)
#include <Windows.h>

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
#include <unistd.h>
#include <sys/resource.h>
#include <sys/times.h>
#include <time.h>

#else
#error "Unable to define getCPUTime( ) for an unknown OS."
#endif





/**
 * Returns the amount of CPU time used by the current process,
 * in seconds, or -1.0 if an error occurred.
 */
double getCPUTime( )
{
#if defined(_WIN32)
    /* Windows -------------------------------------------------- */
    FILETIME createTime;
    FILETIME exitTime;
    FILETIME kernelTime;
    FILETIME userTime;
    if ( GetProcessTimes( GetCurrentProcess( ),
                          &createTime, &exitTime, &kernelTime, &userTime ) != -1 )
    {
        SYSTEMTIME userSystemTime;
        if ( FileTimeToSystemTime( &userTime, &userSystemTime ) != -1 )
            return (double)userSystemTime.wHour * 3600.0 +
                    (double)userSystemTime.wMinute * 60.0 +
                    (double)userSystemTime.wSecond +
                    (double)userSystemTime.wMilliseconds / 1000.0;
    }

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
    /* AIX, BSD, Cygwin, HP-UX, Linux, OSX, and Solaris --------- */

#if defined(_POSIX_TIMERS) && (_POSIX_TIMERS > 0)
    /* Prefer high-res POSIX timers, when available. */
    {
        clockid_t id;
        struct timespec ts;
#if _POSIX_CPUTIME > 0
        /* Clock ids vary by OS.  Query the id, if possible. */
        if ( clock_getcpuclockid( 0, &id ) == -1 )
#endif
#if defined(CLOCK_PROCESS_CPUTIME_ID)
            /* Use known clock id for AIX, Linux, or Solaris. */
            id = CLOCK_PROCESS_CPUTIME_ID;
#elif defined(CLOCK_VIRTUAL)
            /* Use known clock id for BSD or HP-UX. */
            id = CLOCK_VIRTUAL;
#else
            id = (clockid_t)-1;
#endif
        if ( id != (clockid_t)-1 && clock_gettime( id, &ts ) != -1 )
            return (double)ts.tv_sec +
                    (double)ts.tv_nsec / 1000000000.0;
    }
#endif

#if defined(RUSAGE_SELF)
    {
        struct rusage rusage;
        if ( getrusage( RUSAGE_SELF, &rusage ) != -1 )
            return (double)rusage.ru_utime.tv_sec +
                    (double)rusage.ru_utime.tv_usec / 1000000.0;
    }
#endif

#if defined(_SC_CLK_TCK)
    {
        const double ticks = (double)sysconf( _SC_CLK_TCK );
        struct tms tms;
        if ( times( &tms ) != (clock_t)-1 )
            return (double)tms.tms_utime / ticks;
    }
#endif

#if defined(CLOCKS_PER_SEC)
    {
        clock_t cl = clock( );
        if ( cl != (clock_t)-1 )
            return (double)cl / (double)CLOCKS_PER_SEC;
    }
#endif

#endif

    return -1;		/* Failed. */
}
namespace thin_navigation {

using namespace std;


GridPlanner::GridPlanner(){
    _distance_cost_factor=0.5;
    _distance_threshold=2.0;
    _goal.setZero();
    _robot_pose.setZero();
    _heuristic_good = false;
    _plan_found = "";
    _robot_radius=0.3;
    _local_map_dimension=6.0;
    _consider_unknown_as_obstacle=true;
    _bump_cost=1e5;
    _local_offset.y()=0;
    _local_offset.x()=0;
    _goal_tollerance_t=0.5; // TODO params
    _goal_tollerance_r=3.14; // TODO params
    _persistency=300;
    _actual_tv=0;
    _actual_rv=0;
    _compute_global_path=false;
    _thresh_same_point=0.2;
    _heuristic_weight=0.3;
    _nearest_dynamic_object=-1;
}

void GridPlanner::setMap(const UnsignedCharImage& m,
                         float resolution,
                         unsigned char occ_threshold,
                         unsigned char free_threshold) {
    _global.resolution=resolution;
    _global.inverse_resolution = 1./resolution;
    _local.resolution=resolution;
    _local.inverse_resolution = 1./resolution;
    _global.map_image=m.clone();

    // creates an integer matrix where the cells are either
    // -1 (occupied)
    // -2 (unknown)
    // k>0 an unique integer that represent an obstacle in the map.
    _global.int_map.create(m.rows, m.cols);
    _global.rows=m.rows;
    _global.cols=m.cols;
    int k = 0;
    int free_count = 0, occ_count = 0, unknown_count = 0;

    for (int r = 0; r<m.rows; r++){
        unsigned char* src_ptr = _global.map_image.ptr<unsigned char>(r);
        int* dest_ptr = _global.int_map.ptr<int>(r);
        for (int c = 0; c<_global.cols; c++) {
            unsigned char cell=*src_ptr;
            int v=-1;
            if (cell<occ_threshold) {
                occ_count++;
                v=k++;
                *src_ptr=0;
            } else if (cell>free_threshold) {
                free_count++;
                v=-1;
                *src_ptr=255;
            } else {
                if (_consider_unknown_as_obstacle){
                    v=k++;
                } else {
                    unknown_count++;
                    v=-2;
                }
                *src_ptr=127;
            }
            *dest_ptr=v;
            dest_ptr++;
            src_ptr++;
        }
    }
    cerr << "free: " << free_count << endl;
    cerr << "unknown: " << unknown_count << endl;
    cerr << "occupied: " << occ_count << endl;
    //init();
}



void GridPlanner::init(){
    // computes the distance map
    // _int_map is the integer map set through setMap
    // _distances are floats representing the distances (in meters)
    // between a cell and the closest obstacles
    // The distances are computed also for the unknown cells
    // but stored as  their opposite (-d).
    _global.distance_map.compute(_global.assoc_map, _global.distances,
                                 _global.int_map, _distance_threshold*_global.inverse_resolution);
    _global.distances*=_global.resolution;
    _local.width=_local_map_dimension;
    _local.height=_local_map_dimension;
    _local.rows=floor(_local_map_dimension*_global.inverse_resolution);
    _local.cols=floor(_local_map_dimension*_global.inverse_resolution);
    _local.int_map.create(_local.rows, _local.cols);
    _dynamic.int_map.create(_local.rows, _local.cols);

    _local.cost.create(_local.rows,_local.cols);
    search_map.resize(_local.rows, _local.cols);
    _temp_distances.create(_local.rows, _local.cols);
    _local.distances.create(_local.rows, _local.cols);
    _local.heuristic.resize(_local.rows, _local.cols);
    _local.map_image.create(_local.rows, _local.cols);

}

std::string GridPlanner::computePlan() {
    _plan_found="";
    if (! _heuristic_good){
        cout<<"HEURISTIC NOT GOOD"<<endl;
        return "HEURISTIC NOT GOOD";
    }
    _plan_found=planning();
    // cout<<"PLAN FOUND: "<<_plan_found.c_str()<<endl;
    return _plan_found;
}



bool GridPlanner::computeHeuristic() {
    double startTime, endTime;

    startTime = getCPUTime( );

    // if goal ouside map, return false
    if (_goal_rc.x()<0||_goal_rc.x()>=_global.distances.rows||_goal_rc.y()<0||_goal_rc.y()>=_global.distances.cols){
        cout<<"GOAL OUTSIDE MAP"<<endl;
        return false;
    }

    // if goal too close to obstacle, return false
    float d=_global.distances.at<float>(_goal_rc.x(),_goal_rc.y());
    if (d<_robot_radius){
        cout<<"GOAL TOO CLOSE"<<endl;
        return false;
    }
    _global.heuristic.resize(_global.rows, _global.cols);
    for(int c=0; c<_global.cols; c++)
        for (int r=0; r<_global.rows; r++) {
            DistanceMapCell& cell = _global.heuristic(r,c);
            cell.parent=0;
            cell.distance=std::numeric_limits<float>::max();
            cell.r=r;
            cell.c=c;
        }
    DistanceMapCell& goal_cell = _global.heuristic(_goal_rc.x(),_goal_rc.y());
    goal_cell.parent= &goal_cell;
    goal_cell.distance=0;
    
    MyQueue q;
    q.push(&goal_cell);

    while(! q.empty()){
        DistanceMapCell* current=q.top();
        if (current->distance>=q.topDistance()) {
            DistanceMapCell* neighbors[8];
            int num_neighbors = _global.heuristic.findNeighbors(neighbors, current);
            for (int i=0; i<num_neighbors; i++){
                DistanceMapCell* child=neighbors[i];
                float dx=(current->r-child->r)*_global.resolution;
                float dy=(current->c-child->c)*_global.resolution;
                float new_distance=current->distance+std::sqrt(dx*dx+dy*dy)+cost(_global.distances.at<float>(child->r,child->c));
                if (child->distance>new_distance /*&& new_distance<_bump_cost*/){
                    child->parent=current;
                    child->distance=new_distance;
                    q.push(child);
                }
            }
        }
        q.pop();
    }

    _local_plan.clear();
    return true;
    endTime = getCPUTime( );
    PRINT_DEBUG("Compute Heuristic"<<(endTime - startTime));

}

std::string GridPlanner::planning() {

    double startTime, endTime;

    startTime = getCPUTime( );

    // trivial function to compute a path
    float goal_distance=0;
    // retrieve the row and the column of the robot in the grid
    Eigen::Vector3f _current_robot_pose=_robot_pose;
    Eigen::Vector3f _current_goal=_goal;
    Eigen::Vector2i _current_robot_pose_rc=_robot_pose_rc;
    Eigen::Vector2i _current_goal_rc=_goal_rc;


    if (_current_robot_pose_rc.x()<0||_current_robot_pose_rc.x()>=_global.rows){
        cout<<"ROBOT ROW OUT OF MAP - "<<_current_robot_pose_rc.x()<<"max rows"<<_global.rows<<endl;
        return "ROBOT ROW OUT OF MAP";
    }
    if (_current_robot_pose_rc.y()<0||_current_robot_pose_rc.y()>=_global.cols){
        cout<<"ROBOT COL OUT OF MAP - "<<_current_robot_pose_rc.y()<<" - "<<_global.cols<<endl;
        return "ROBOT COL OUT OF MAP";
    }
    if (_global.heuristic(_current_robot_pose_rc.x(),_current_robot_pose_rc.y()).distance>=std::numeric_limits<float>::max()){
        cout<<"UNREACHBLE GOAL"<<endl;
        return "UNREACHBLE GOAL";
    }
    if(abs(_current_robot_pose.x()-_current_goal.x())<_goal_tollerance_t && abs(_current_robot_pose.y()-_current_goal.y())<_goal_tollerance_t)
    {
        if((_current_robot_pose.z() > _current_goal.z()-_goal_tollerance_r && _current_robot_pose.z() < _current_goal.z()+_goal_tollerance_r)){
            //cout<<"SUCCEEDED"<<endl;
            return "SUCCEEDED";

        }else{
            //pure rotation
            _local_plan.clear();
            Eigen::Vector4f pose_r(_current_robot_pose.x(), _current_robot_pose.y(), _current_robot_pose.z(),1);
            _local_plan.push_back(pose_r);
            Eigen::Vector4f pose_g(_current_goal.x(), _current_goal.y(), _current_goal.z(),1);
            // Eigen::Vector4f pose_g(_current_robot_pose.x(), _current_robot_pose.y(), _current_goal.z(),1);
            _local_plan.push_back(pose_g);
            //cout<<"GO"<<endl;
            //PRINT_DEBUG("PURE ROTATION"<<endl);
            return "ROTATION";

        }

    }

    Vector4fVector _temp_local_plan;
    DistanceMapCell goal;
    bool near=false;
    bool reached=false;

    if ((_current_goal_rc.x()<_local_offset.x() +_local.rows && _current_goal_rc.x()>_local_offset.x())&&
            (_current_goal_rc.y()<_local_offset.y()+_local.cols && _current_goal_rc.y()>_local_offset.y())){
        //cout<<"NEAR GOAL"<<endl;
        goal.distance = 0;
        goal.r=_current_goal_rc.x()-_local_offset.x();
        goal.c=_current_goal_rc.y()-_local_offset.y();
        goal_distance=_local.distances(goal.r,goal.c);
        near=true;
    }else{
        float min=std::numeric_limits<float>::max();
        for(int r=0; r< _local.rows; r++){
            for( int c=0; c< _local.cols; c++){
                if ((float)_local.heuristic(r,c).distance < min  ){
                    min= _local.heuristic(r,c).distance;
                    goal.distance = _local.heuristic(r,c).distance;
                    goal.c=c;
                    goal.r=r;
                }
            }
        }
    }
    if(_nearest_dynamic_object!=-1){
        reached=search(_local.distances, _local.heuristic, goal, _temp_local_plan);
        if (! reached){
            return "UNREACHEBLE GOAL";
        }
        if ( !_temp_local_plan.size()){
            return "SUCCEEDED";
        }

        //after the last point of the plan on the local map, fill the plan with the global path
        if (_temp_local_plan.size()>=1) {
            Eigen::Vector2i last_plan_pose=world2grid(Eigen::Vector2f(_temp_local_plan[_temp_local_plan.size()-1].head(2)));
            DistanceMapCell* current=&_global.heuristic(last_plan_pose.x(),last_plan_pose.y());
            reconstructPath(current,_temp_local_plan,_global.distances,Eigen::Vector2i(0,0));

        }
    }else{
        DistanceMapCell* current=&_global.heuristic(_robot_pose_rc.x(),_robot_pose_rc.y());
        reconstructPath(current,_temp_local_plan,_global.distances,Eigen::Vector2i(0,0));
    }
    if(_compute_global_path){
        _global_plan.clear();
        DistanceMapCell* current=&_global.heuristic(_robot_pose_rc.x(),_robot_pose_rc.y());
        reconstructPath(current,_global_plan,_global.distances,Eigen::Vector2i(0,0));
    }
    //cout<<"FILL THE ANGLES"<<endl;
    // fill the angles
    _temp_local_plan[0].z()=_current_robot_pose.z();
    for (int i=1; i<(int)_temp_local_plan.size()-1; i++){
        Eigen::Vector4f prev_pose=_temp_local_plan[i-1];
        Eigen::Vector4f& curr_pose=_temp_local_plan[i];
        Eigen::Vector4f next_pose=_temp_local_plan[i+1];
        Eigen::Vector2f delta=next_pose.head<2>()-prev_pose.head<2>();
        float angle=std::atan2(delta.y(), delta.x());
        curr_pose.z()=angle;

    }


    _temp_local_plan[_temp_local_plan.size()-1].x()=_current_goal.x();
    _temp_local_plan[_temp_local_plan.size()-1].y()=_current_goal.y();
    _temp_local_plan[_temp_local_plan.size()-1].z()=_current_goal.z();
    _temp_local_plan[_temp_local_plan.size()-1].w()=goal_distance;


    _local_plan=_temp_local_plan;
    endTime = getCPUTime( );
    PRINT_DEBUG("Planning"<<(endTime - startTime));
    //return "GO";
    if(near){
        //cout<<"NEAR"<<endl;
        return "NEAR";
    }else{
        //cout<<"GO"<<endl;
        return "GO";
    }
}



bool GridPlanner::search(const FloatImage &map, DistanceMap &heuristic, const DistanceMapCell &goal, Vector4fVector &plan)
{

    //    FloatImage prova;
    //    prova.create(map.rows, map.cols);
    double startTime, endTime;

    startTime = getCPUTime( );
    for(int c=0; c<search_map.cols(); c++){
        for (int r=0; r<search_map.rows(); r++) {
            DistanceMapCell& cell = search_map(r,c);
            cell.parent=0;
            cell.distance=std::numeric_limits<float>::max();
            cell.weight=heuristic(r,c).distance;
            cell.r=r;
            cell.c=c;
            //            prova(r,c)=0;
        }
    }

    DistanceMapCell& start_cell = search_map(_local_pose_rc.x(),_local_pose_rc.y());
    start_cell.parent= &start_cell;
    start_cell.distance=0;
    MyQueue q;
    DistanceMapCell* current;
    q.push(start_cell.distance+start_cell.weight, &start_cell);
    float t=_goal_tollerance_t*_global.inverse_resolution;
    MyQueue goal_queue;
    int da_cancellare=0;

    while (! q.empty()){
        da_cancellare++;
        current=q.top();
        if (current->weight>=q.topDistance()) {

            if ((current->c > goal.c-t && current->c < goal.c+t)
                    && (current->r > goal.r-t && current->r < goal.r+t))
            {
                DistanceMapCell& cell_goal = search_map(current->r,current->c);
                goal_queue.push(heuristic(current->r,current->c).distance,&cell_goal);
                break;
            }else{
                if ((current->c == 0) || (current->c==search_map.cols()-1) || (current->r == 0) || (current->r==search_map.rows()-1))
                {
                    DistanceMapCell& cell_goal = search_map(current->r,current->c);
                    goal_queue.push(heuristic(current->r,current->c).distance,&cell_goal);
                }
            }

            DistanceMapCell* neighbors[8];
            int num_neighbors = search_map.findNeighbors(neighbors, current);
            for (int i=0; i<num_neighbors; i++){
                DistanceMapCell* child=neighbors[i];
                float dx=(current->r-child->r)*_global.resolution;
                float dy=(current->c-child->c)*_global.resolution;
                float step=std::sqrt(dx*dx+dy*dy);

                float new_distance=current->distance+step+_local.cost(child->r,child->c);
                if(child->distance > new_distance){
                    if((_local.cost(current->r, current->c)<_bump_cost) ||
                            (_local.cost(current->r, current->c) >=_bump_cost &&
                             _local.distances(current->r, current->c)<_local.distances(child->r, child->c))){
                        child->parent=current;
                        child->distance=new_distance;
                        child->weight=child->distance+(heuristic(child->r,child->c).distance*_heuristic_weight);
                        q.push(child->weight,child);
                        //                        prova(child->r,child->c)=100;
                    }
                }

            }
        }
        q.pop();
        //                     if((da_cancellare%10)==0){
        //                      cv::circle(prova,cv::Point(goal.c,goal.r),3,cv::Scalar(120),-1);
        //                      cv::imshow("prova",prova);
        //                      cv::waitKey(1);
        //                }
    }

    current= goal_queue.top();
    if(current==NULL){
        return false;
    }

    if(heuristic(current->r,current->c).distance>heuristic(_local_pose_rc.x(),_local_pose_rc.y()).distance){
        return false;
    }
    reconstructInversePath(current,plan,map);
    endTime = getCPUTime( );
    PRINT_DEBUG("Search"<<(endTime - startTime));
    return true;


}

void GridPlanner::reconstructInversePath(DistanceMapCell* start,  Vector4fVector& plan,FloatImage map){
    Vector4fVector reverse_plan;
    reverse_plan.clear();
    reconstructPath(start,reverse_plan,map,_local_offset);
    int plan_size=reverse_plan.size();

    plan.resize(plan_size);
    for (int i=0; i<plan_size;i++)
        plan[i]=reverse_plan[plan_size-1-i];
}

void GridPlanner::reconstructPath(DistanceMapCell* start,  Vector4fVector& plan,FloatImage map,Eigen::Vector2i offset){
    DistanceMapCell* cell=start;
    while(cell->parent && cell!=cell->parent){
        Eigen::Vector4f pose((cell->r +offset.x())*_global.resolution, (cell->c+offset.y())*_global.resolution, 0,map(cell->r,cell->c));
        plan.push_back(pose);
        cell=cell->parent;
    }

}

void GridPlanner::computeEndPoints(const Vector2fVector &observation)
{    double startTime, endTime;

     startTime = getCPUTime( );
      _last_endpoints = observation;

       //Calcolo la trasformata tra la posa precedente del robot e quella attuale
       Eigen::Isometry2f _new_pose_transform=v2t(_robot_pose);
        Eigen::Isometry2f _old_pose_transform=v2t(_old_robot_pose);

         Eigen::Isometry2f _t=_new_pose_transform.inverse()*_old_pose_transform;
          _old_robot_pose=_robot_pose;

           std::vector<std::pair<float,float> > _polar_point(65);
            for (size_t i=0; i<_last_endpoints.size();i++){
                float r,t;
                polar(_last_endpoints[i].x(),_last_endpoints[i].y(),r,t);
                int index=round((t+M_PI)*10);
                if (index<0){
                    index=0;
                }

                _polar_point[index]=std::make_pair(r,_persistency);

            }

             Vector3fVector temp_endpoints(_prec_endpoints.size());

              int n_point=0;
               if (_prec_endpoints.size()>0)
                   for(size_t i=0; i<_prec_endpoints.size();i++){
                       _prec_endpoints[i].z()-=1;
                       if(_prec_endpoints[i].z()>0){
                           float r,t;
                           _prec_endpoints[i].head<2>()=_t*_prec_endpoints[i].head<2>();
                           polar(_prec_endpoints[i].x(),_prec_endpoints[i].y(),r,t);
                           int index=round((t+M_PI)*10);
                           if (index<0){
                               index=0;
                           }
                           if ( _polar_point[index].first < r-_thresh_same_point ){
                               temp_endpoints[n_point]=_prec_endpoints[i];
                               n_point++;
                           }
                       }
                   }

               _prec_endpoints.resize(n_point+_last_endpoints.size());
                for (size_t i=0;i<_last_endpoints.size();i++){
                    _prec_endpoints[i]=Eigen::Vector3f(_last_endpoints[i].x(),_last_endpoints[i].y(),_persistency);
                }
                 for (size_t i=_last_endpoints.size();i<_prec_endpoints.size();i++){
                     _prec_endpoints[i]=temp_endpoints[i-_last_endpoints.size()];
                 }
                  endTime = getCPUTime( );
                   PRINT_DEBUG("Compute endpoints: "<<(endTime - startTime));
}

void GridPlanner::updateTemporaryMap(const Vector2fVector& observation){

    computeEndPoints(observation);
    double startTime, endTime;

    startTime = getCPUTime( );
    _local_offset.x()=_robot_pose_rc.x()-_local.rows/2;
    _local_offset.y()=_robot_pose_rc.y()-_local.cols/2;

    _local.int_map=-1;

    _local_pose = _robot_pose;
    _local_pose.x()=_local_map_dimension/2;
    _local_pose.y()=_local_map_dimension/2;
    _local_pose_rc.x()=round(_robot_pose.x()*_global.inverse_resolution-_local_offset.x());
    _local_pose_rc.y()=round(_robot_pose.y()*_global.inverse_resolution-_local_offset.y());


    int endpoint_id=_global.rows*_global.cols;
    Eigen::Isometry2f robot_pose_transform=v2t(_local_pose);


    for (size_t i=0; i<_prec_endpoints.size(); i++){
        Eigen::Vector2f ep=robot_pose_transform*Eigen::Vector2f(_prec_endpoints[i].x(),_prec_endpoints[i].y());
        int r = ep.x()*_global.inverse_resolution;
        int c = ep.y()*_global.inverse_resolution;
        if (r<0||r>=_local.rows||c<0||c>=_local.cols)
            continue;
        _local.int_map.at<int>(r,c)=endpoint_id;
        endpoint_id++;
    }
    endTime = getCPUTime( );
    PRINT_DEBUG("Update Temporary map (prec_endpoints): "<<(endTime - startTime));
    startTime=endTime;
    // cout<<"TEMP INT MAT SIZE ROWS: "<<_local_int_map.rows<<" COLS: "<<_local_int_map.cols<<endl;
    // _local.distance_map.compute(_local.assoc_map, _local.distances,
    //                             _local.int_map, _distance_threshold*_global.inverse_resolution);

    //cout<<"TEMP INT MAT SIZE AFTER COMPUTE ROWS: "<<_local_int_map.rows<<" COLS: "<<_local_int_map.cols<<endl;
    _local.distances*=_global.resolution;
    endTime = getCPUTime( );
    PRINT_DEBUG("Update Temporary map (local_distance_map): "<<(endTime - startTime));
    startTime=endTime;
    if (_heuristic_good){


        for (int r=0; r<_local.rows;r++){
            for (int c=0; c<_local.cols; c++){
                if(r+_local_offset.x()<0 || c+_local_offset.y()<0 ||(c+_local_offset.y())>=_global.cols
                        || (r+_local_offset.x())>=_global.rows){
                    _local.heuristic(r,c).distance=_bump_cost;
                    _temp_distances(r,c)= 0;
                }else{
                    _local.heuristic(r,c).distance = _global.heuristic(r+_local_offset.x(),c+_local_offset.y()).distance;
                    _temp_distances(r,c)=_global.distances(r+_local_offset.x(),c+_local_offset.y());
                }

            }
        }

        //        cv::absdiff(_temp_distances,_local.distances,_local.cost);
        // _local.distances=cv::min(_temp_distances,_local.distances);
        //        cv::imshow("dynamic obstacle",_local.cost);

        endTime = getCPUTime( );
        PRINT_DEBUG("Update Temporary map (temp_distances): "<<(endTime - startTime));
        startTime=endTime;

        float nearest_obs=std::numeric_limits<float>::max();
        bool ob=false;
        for (int r=0; r<_local.rows;r++){
            for (int c=0; c<_local.cols; c++){
                if(r+_local_offset.x()<0 || c+_local_offset.y()<0 ||(c+_local_offset.y())>=_global.cols
                        || (r+_local_offset.x())>=_global.rows){
                    _dynamic.int_map(r,c)= 0;
                }else{
                    if(_local.int_map.at<int>(r,c)>0 && _global.distances(r+_local_offset.x(),c+_local_offset.y())>0.2){
                        _dynamic.int_map(r,c)=_local.int_map.at<int>(r,c);
                        float dx=(r-_local_pose_rc.x())*_global.resolution;
                        float dy=(c-_local_pose_rc.y())*_global.resolution;;
                        float dist=std::sqrt(dx*dx+dy*dy);
                        if(dist<nearest_obs){
                            nearest_obs=dist;
                            ob=true;
                        }


                    }else{
                        _dynamic.int_map(r,c)=-1;

                    }
                }

            }

        }

        if(ob){
            _nearest_dynamic_object=nearest_obs;
        }else{
            _nearest_dynamic_object=-1;
        }
        _dynamic.distance_map.compute(_dynamic.assoc_map, _dynamic.distances,
                                      _dynamic.int_map, 2*_distance_threshold*_global.inverse_resolution);
        _dynamic.distances*=_global.resolution;
        _local.distances=cv::min(_temp_distances,_dynamic.distances);
        endTime = getCPUTime( );
        PRINT_DEBUG("Update Temporary map (dynamic_distances): "<<(endTime - startTime));
        startTime=endTime;
        for (int r=0; r<_local.rows;r++){
            for (int c=0; c<_local.cols; c++){
                _local.cost(r,c)=(cost(_local.distances(r,c))+cost(_dynamic.distances(r,c)));
                //                _local.cost(r,c)=cost(_local.distances(r,c));
            }
        }
        //        cout<<"cost where i am: "<<_local.cost(_local_pose_rc.x(),_local_pose_rc.y())<<endl;
        //        cout<<"distance where i am: "<<_local.distances(_local_pose_rc.x(),_local_pose_rc.y())<<endl;


    }
    endTime = getCPUTime( );
    PRINT_DEBUG("Update Temporary map(local_cost): "<<(endTime - startTime));
}

void GridPlanner::paintState(RGBImage& img, bool use_distance_map){

    if (! use_distance_map) {
        cvtColor(_global.map_image, img, CV_GRAY2BGR);
    } else {
        UnsignedCharImage dest;

        // _global.heuristic.toImage(dest);
        _global.distance_map.toImage(dest);
        cvtColor(dest, img , CV_GRAY2BGR);
    }
    //_local.distance_map.toImage(_local.map_image);

    float ires=1./_global.resolution;
    Eigen::Isometry2f robot_pose_transform=v2t(_robot_pose);

    for (size_t i=0; i<_last_endpoints.size(); i++){
        Eigen::Vector2f ep=robot_pose_transform*_last_endpoints[i];
        int r = ep.x()*ires;
        int c = ep.y()*ires;
        if (r<0||r>=img.rows||c<0||c>=img.cols)
            continue;
        cv::Scalar color(255,0,0);
        cv::circle(img, cv::Point(c,r), 2, color);

    }

    // paint the path
    if (isPlanFound())
    {   _local.map_image=(_local.distances*-255)+255;
        cv::Scalar color(255,0,0);
        int prev_r=_robot_pose.x()*ires;
        int prev_c=_robot_pose.y()*ires;
        for(size_t i=0; i<_local_plan.size(); i++){
            const Eigen::Vector4f& pose=_local_plan[i];
            // retrieve the row  and column of the goal in the map
            int pose_r = pose.x()*ires;
            int pose_c = pose.y()*ires;
            if (pose_r>_local_offset.x() && pose_r < _local_offset.x()+_local.rows
                    && pose_c>_local_offset.y() && pose_c < _local_offset.y()+_local.cols ){
                _local.map_image.at<unsigned char>(pose_r-_local_offset.x(),pose_c-_local_offset.y())=254;
            }
            cv::line(img, cv::Point(pose_c,pose_r), cv::Point(prev_c,prev_r), cv::Scalar(255,255,0));
            prev_r=pose_r;
            prev_c=pose_c;
        }

        if(true){
            float step=50;
            Vector3fVector trajectory(step);
            trajectory[0]=Eigen::Vector3f(0,0,0);
            for(int i=1;i<step;i++){
                trajectory[i].x()=trajectory[i-1].x()+_actual_tv*(1/step)*cos(trajectory[i-1].z()+(_actual_rv*(1/step))/2);
                trajectory[i].y()=trajectory[i-1].y()+_actual_tv*(1/step)*sin(trajectory[i-1].z()+(_actual_rv*(1/step))/2);
                trajectory[i].z()=trajectory[i-1].z()+_actual_rv*(1/step);
            }
            Eigen::Isometry2f orig_pose=v2t(_local_pose);
            for(int i=0;i<step;i++){

                Eigen::Isometry2f robot_point=v2t(trajectory[i]);
                Eigen::Vector3f local_map_pose=t2v(orig_pose*robot_point);
                int r=local_map_pose.x()*_global.inverse_resolution;
                int c=local_map_pose.y()*_global.inverse_resolution;
                if(r<_local.rows && c<_local.cols && r>0 && c>0){
                    _local.map_image.at<unsigned char>(r,c)=254;
                }
            }
            Eigen::Isometry2f path_point=v2t(_next_point);
            Eigen::Vector3f local_map_point=t2v(orig_pose*path_point);
                    int r=local_map_point.x()*_global.inverse_resolution;
                    int c=local_map_point.y()*_global.inverse_resolution;
            //int r=_next_point.x()*_global.inverse_resolution;
            //int c=_next_point.y()*_global.inverse_resolution;

            cv::Scalar color(255,0,0);
            cv::circle(_local.map_image, cv::Point(c,r), 5, color);
        }
        cv::Point center = cv::Point( _local.cols/2, _local.rows/2 );
        double angle = 90.0;
        double scale = 1;
        /// Rotate the warped image
        warpAffine( _local.map_image, _local.map_image, getRotationMatrix2D( center, angle, scale ), _local.map_image.size() );
    }

}


}
