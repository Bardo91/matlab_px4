cmake_minimum_required(VERSION 3.8 FATAL_ERROR)


project(wp_calculation VERSION 0.1 LANGUAGES C CXX)

find_package(PCL REQUIRED)
find_package(motion_planning REQUIRED)
find_package(catkin REQUIRED roscpp geometry_msgs)

add_executable(wp_calculation main.cpp)
target_compile_features(wp_calculation PUBLIC cxx_std_11)
target_link_libraries(wp_calculation LINK_PUBLIC motion_planning::motion_planning)

target_include_directories(wp_calculation PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(wp_calculation LINK_PUBLIC ${catkin_LIBRARIES})

target_include_directories(wp_calculation PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(wp_calculation LINK_PUBLIC ${PCL_LIBRARIES})

