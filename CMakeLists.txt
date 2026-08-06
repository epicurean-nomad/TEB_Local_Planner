cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Default to Release when no build type is specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()


project(teb_local_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(EXISTS "${CMAKE_SOURCE_DIR}/CMakeLocalConfig.cmake")
    include("${CMAKE_SOURCE_DIR}/CMakeLocalConfig.cmake")
endif()

set(G2O_ROOT "${G2O_ROOT}" CACHE PATH "Path to g2o clone root")
set(G2O_BUILD_LIB_DIR "${G2O_ROOT}/build/lib" CACHE PATH "Path to g2o build lib dir")



find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(test_local_planner_viz
    src/test_local_planner_viz.cpp
    aux/RoutingLayer.cpp
    aux/AreaGraph.cpp
    aux/RoadSegment.cpp
    aux/Junction.cpp
    aux/Serializer.cpp
    aux/APSP.cpp
)
target_include_directories(test_local_planner_viz PRIVATE
    ${CMAKE_SOURCE_DIR}/src lcm_types 
    ${G2O_ROOT}                # adjust to wherever your g2o clone root is
    ${OpenCV_INCLUDE_DIRS}
)
target_link_directories(test_local_planner_viz PRIVATE
    ${G2O_BUILD_LIB_DIR}      # adjust to your g2o build dir
)
target_link_libraries(test_local_planner_viz PRIVATE
    g2o_core g2o_stuff g2o_solver_eigen g2o_types_slam2d
    ${OpenCV_LIBS} Threads::Threads Eigen3::Eigen
)

set_target_properties(test_local_planner_viz PROPERTIES
    BUILD_RPATH "${G2O_BUILD_LIB_DIR}"
    INSTALL_RPATH "${G2O_BUILD_LIB_DIR}"
)

install(TARGETS test_local_planner_viz DESTINATION lib/${PROJECT_NAME})



