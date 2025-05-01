cmake_minimum_required(VERSION 3.0.2)
project(bwt_ssg_builder)

find_package(catkin_simple REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(predictive_planner_common REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -O3 -Werror=return-type) # -O3

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/bwt_ssg_builder.cpp
  src/bwt_ssg_manager.cpp
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# cs_add_executable(subdue_cpp_node src/main.cpp)
# target_link_libraries(subdue_cpp_node ${PROJECT_NAME} GSL::gsl GSL::gslcblas)

##########
# EXPORT #
##########
cs_install()
cs_export(LIBRARIES)
