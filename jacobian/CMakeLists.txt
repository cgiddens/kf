cmake_minimum_required(VERSION 3.5.0)
project(Jacobian)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/../eigen/Eigen
    )

add_executable(jacobian jacobian.cpp
    )

set( CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR} )
#install(FILES EKF_DATA/obj_pose-laser-radar-synthetic-input.txt DESTINATION build/)
