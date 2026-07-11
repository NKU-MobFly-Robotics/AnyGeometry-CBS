cmake_minimum_required(VERSION 3.10)
project(any_geometry_cbs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    include/any_geometry_cbs
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/third_party/libMultiRobotPlanning/include/libMultiRobotPlanning
    ${PROJECT_SOURCE_DIR}/third_party/libMultiRobotPlanning/include
)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/libMultiRobotPlanning)

add_executable(AG_CBS src/AG_CBS.cpp)
target_link_libraries(AG_CBS 
    ${Boost_LIBRARIES} 
    yaml-cpp
)

add_executable(AG_ECBS src/AG_ECBS.cpp)
target_link_libraries(AG_ECBS 
    ${Boost_LIBRARIES} 
    yaml-cpp
)

install(TARGETS AG_CBS AG_ECBS
  DESTINATION lib/${PROJECT_NAME}
)
