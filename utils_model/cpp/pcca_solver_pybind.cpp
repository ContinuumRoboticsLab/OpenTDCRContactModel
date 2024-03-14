//
// Created by Itai on 20/11/2023.
//

#include <pybind11/pybind11.h>
#define FORCE_IMPORT_ARRAY
#include <pybind11/stl.h>
#include <xtensor-python/pytensor.hpp>

#include "pcca_solver_pytensor.h"

PYBIND11_MODULE(pcca_solver, m) {
    xt::import_numpy();
    pybind11::class_<pcca_solver_pytensor>(m, "pcca_solver")
            .def(pybind11::init())
            .def("solve", &pcca_solver_pytensor::solve);
}