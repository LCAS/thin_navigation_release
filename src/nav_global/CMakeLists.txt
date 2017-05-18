add_library(thin_navigation_nav_global_library SHARED
  distance_map.cpp distance_map.h
  )

target_link_libraries(thin_navigation_nav_global_library
  ${OpenCV_LIBS} 
)



