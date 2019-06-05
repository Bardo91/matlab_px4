cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(data_visualizer VERSION 0.1 LANGUAGES C CXX)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs)

find_package(PCL REQUIRED )

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE HEADER_FILES "include/*.h" "include/*.inl" "include/*.hpp")

add_executable(${PROJECT_NAME} ${SOURCE_FILES} ${HEADER_FILES})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_11)

target_include_directories(${PROJECT_NAME} PUBLIC include)
target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PCL_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})