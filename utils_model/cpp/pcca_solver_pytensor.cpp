//
// Created by Itai on 21/11/2023.
//

#include "pcca_solver_pytensor.h"

#include "pcca_solver_nlopt.h"

#include <algorithm>
#include <utility>
#include <xtensor/xarray.hpp>

pcca_solver_pytensor::pcca_solver_pytensor() : solver(new pcca_solver_nlopt()){
}

std::tuple<std::vector<double>, int> pcca_solver_pytensor::solve(int n, double l, double l_tendon, double r, std::vector<double> x_init,
                                                                 const xt::pytensor<double, 2>& tendon, const xt::pyarray<double>& ab,
                                                                 const xt::pyarray<double>& s, const xt::pyarray<double>& c_center) {

    return solver->solve(n, l, l_tendon, r, std::move(x_init), tendon, ab, s, c_center);
}
