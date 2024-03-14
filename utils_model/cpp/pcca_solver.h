//
// Created by Itai on 21/11/2023.
//

#ifndef CONTACT_MODEL_CPP2_PCCA_SOLVER_H
#define CONTACT_MODEL_CPP2_PCCA_SOLVER_H

#include <vector>
#include <xtensor/xtensor.hpp>

class pcca_solver {
public:
    virtual std::tuple<std::vector<double>, int> solve(int n, double l, double l_tendon, double r, std::vector<double> x_init,
                                               const xt::xtensor<double, 2>& tendon, const xt::xarray<double>& ab,
                                               const xt::xarray<double>& s, const xt::xarray<double>& c_center) = 0;
    virtual ~pcca_solver() = default;
};


#endif //CONTACT_MODEL_CPP2_PCCA_SOLVER_H
