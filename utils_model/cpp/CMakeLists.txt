cmake_minimum_required(VERSION 3.22) # arbitrary, probably most versions work just as well
project(contact_model_cpp)
#
set(CMAKE_CXX_STANDARD 20)

# for xtensor
add_definitions(-DHAVE_CBLAS=1)
find_package(xtl REQUIRED)
find_package(xtensor REQUIRED)

# Uncomment one of the lines below to choose a BLAS implementation
 set(BLA_VENDOR OpenBLAS)
#set(BLA_VENDOR Intel10_64lp)

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(xtensor-blas REQUIRED)

message(STATUS "BLAS VENDOR:    " ${BLA_VENDOR})
message(STATUS "BLAS LIBRARIES: " ${BLAS_LIBRARIES})

# for python integration
find_package(Python 3.10 REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(pybind11 REQUIRED CONFIG)
find_package(xtensor-python REQUIRED)

# Find nlopt/ipopt using "Find{ipopt/nlopt}.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(nlopt REQUIRED)

add_library(contact_model_cpp SHARED pcca.cpp pcca.h
        pcca_solver.h
        model_utils.cpp
        model_utils.h
        pcca_solver_nlopt.cpp
        pcca_solver_nlopt.h)

target_link_libraries(contact_model_cpp nlopt ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} xtensor xtensor::optimize xtensor::use_xsimd xtensor-blas)

pybind11_add_module(pcca_solver pcca_solver_pybind.cpp pcca_solver_pytensor.cpp pcca_solver_pytensor.h)
target_link_libraries(pcca_solver PRIVATE contact_model_cpp pybind11::module xtensor-python Python::NumPy)