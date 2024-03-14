//
// Created by Itai on 23/11/2023.
//

#ifndef CONTACT_MODEL_CPP_PCCA_SOLVER_NLOPT_H
#define CONTACT_MODEL_CPP_PCCA_SOLVER_NLOPT_H

#include "pcca_solver.h"
#include "nlopt.hpp"

class pcca_solver_nlopt : public pcca_solver {
public:
    pcca_solver_nlopt();
    std::tuple<std::vector<double>, int> solve(int n, double l, double l_tendon, double r, std::vector<double> x_init,
                                               const xt::xtensor<double, 2> &tendon,
                                               const xt::xarray<double> &ab,
                                               const xt::xarray<double> &s,
                                               const xt::xarray<double> &c_center) override;
};


#endif //CONTACT_MODEL_CPP_PCCA_SOLVER_NLOPT_H
