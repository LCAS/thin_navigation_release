add_library(thin_navigation_ros_utils_library 
  ros_utils.cpp ros_utils.h
  )

target_link_libraries(thin_navigation_ros_utils_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS} 
)


install(TARGETS thin_navigation_ros_utils_library
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

