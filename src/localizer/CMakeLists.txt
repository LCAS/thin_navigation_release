add_library(thin_navigation_localizer_library  SHARED
  localization_filter.cpp 
  localization_filter.h
  )

target_link_libraries(thin_navigation_localizer_library
  thin_navigation_nav_global_library
  ${OpenCV_LIBS} 
)

add_executable(thin_localizer_node
  ros_localizer.cpp ros_localizer.h
  thin_localizer_node.cpp
  )

target_link_libraries(thin_localizer_node
  thin_navigation_localizer_library
  thin_navigation_ros_utils_library
  thin_navigation_nav_global_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)
