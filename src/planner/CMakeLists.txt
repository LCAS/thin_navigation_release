add_library(thin_navigation_planner_library 
  grid_planner.cpp grid_planner.h
  )

target_link_libraries(thin_navigation_planner_library
  thin_navigation_nav_global_library
  ${OpenCV_LIBS} 
)

add_executable(thin_planner_node 
  gradient_controller.cpp
  gradient_controller.h
  ros_planner.cpp 
  ros_planner.h
  thin_planner_node.cpp
)
target_link_libraries(thin_planner_node
  thin_navigation_planner_library
  thin_navigation_nav_global_library
  thin_navigation_ros_utils_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)


install(TARGETS thin_navigation_planner_library thin_planner_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
