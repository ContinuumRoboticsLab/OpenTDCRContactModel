cmake_minimum_required(VERSION 3.22) # arbitrary, probably most versions work just as well
project(contact_model_cpp)
#
set(CMAKE_CXX_STANDARD 20)

# for GTest
find_package(GTest CONFIG OPTIONAL_COMPONENTS)

# for google benchmark
find_package(benchmark OPTIONAL_COMPONENTS)

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

add_executable(contact_model_cpp_main main.cpp)
target_link_libraries(contact_model_cpp_main contact_model_cpp)

if(GTest_FOUND)
    add_executable(pcca_tests pcca_tests.cpp)
    target_link_libraries(pcca_tests contact_model_cpp GTest::gtest_main)

    include(GoogleTest)
    gtest_discover_tests(pcca_tests)
endif()

if(benchmark_FOUND)
    add_executable(pcca_benchmarks pcca_bench.cpp)
    target_link_libraries(pcca_benchmarks contact_model_cpp benchmark::benchmark_main)
endif()

pybind11_add_module(pcca_solver pcca_solver_pybind.cpp pcca_solver_pytensor.cpp pcca_solver_pytensor.h)
target_link_libraries(pcca_solver PRIVATE contact_model_cpp pybind11::module xtensor-python Python::NumPy)