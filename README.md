                   
# thin_navigation package

by Giorgio Grisetti, Cristiano Gennari, Luca Iocchi, 2015

[Lab RoCoCo](labrococo.dis.uniroma1.it)

Dept. of Computer, Control and Management Engineering

Sapienza University of Rome, Italy

----

## QUICK README ##

thin_navigation package contains two ROS nodes: thin_localizer and thin_planner.
Download the folder in your ROS catkin workspace and compile with catkin_make.

## Short description and how to use ##

thin_localizer is lightweight variant to amcl
thin_planner is lightweight variant to move_base

They have the same interfaces as amcl and move_base, so they can be used
in their replacement with minimal effort (just change the launch file).

Examples of use in launch file:

    <node pkg="thin_navigation" type="thin_localizer_node" name="thin_localizer" output="screen">
      <param name="initial_pose_x" value="22.8"/>
      <param name="initial_pose_y" value="21.975"/>
      <param name="initial_pose_theta" value="1.57"/>
      <param name="global_frame_id" value="map"/>
      <param name="base_frame_id" value="base_link"/>
      <param name="odom_frame_id" value="odom"/>
      <param name="laser_topic" value="base_scan"/>
    </node>
    
    <node pkg="thin_navigation" type="thin_planner_node" name="thin_planner" output="screen">
      <param name="max_range" value="10.0"/>
      <param name="max_tv" value="1.0"/>
      <param name="max_rv" value="1.0"/>
      <param name="global_frame_id" value="map"/>
      <param name="base_frame_id" value="base_link"/>
      <param name="laser_topic" value="base_scan"/>
      <param name="command_vel_topic" value="cmd_vel"/>
      <param name="robot_radius" value="0.25"/>
      <param name="distance_threshold" value="2.0"/>
    </node>


## Test with stage simulator ##

### Single robot test ###

    $ cd test
    $ ./run_test.sh
 
### Multi robot test ###

    $ cd test
    $ ./run_MR_test.sh

Use rviz to set navigation goals and see the results of localization and navigation.

To quit the simulation use:

    $ rosnode kill -a



## Additional instructions ##

### thin_localizer ###

If you use the -gui flag a window shows the state of the localizer.

You can still control the localizer through ros messages (e.g. via rviz) in the usual way.

Move the robot with a joystick or some other teleoperation means.

Once the GUI starts, here are the keys to use:

    s: with this you toggle the set-pose mode. 
       left click sets the robot position
       right click sets the robot orientation

    g: triggers global localization

    d: toggles the the distance map vs occupancy map view

    r: toggles particle resetting (they get resampled when entering in the unknown)