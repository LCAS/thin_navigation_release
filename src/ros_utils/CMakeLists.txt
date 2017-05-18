add_library(thin_navigation_ros_utils_library 
  ros_utils.cpp ros_utils.h
  )

target_link_libraries(thin_navigation_ros_utils_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)



