//
// Created by Itai on 11/11/2023.
//

#ifndef CONTACT_MODEL_CPP2_MODEL_UTILS_H
#define CONTACT_MODEL_CPP2_MODEL_UTILS_H

#include "xtensor/xarray.hpp"

namespace model_utils {
    xt::xtensor<double,2> trans_mat(double k, double l);
    xt::xtensor<double,2> trans_mat_derivative(double k, double l);
}

#endif //CONTACT_MODEL_CPP2_MODEL_UTILS_H
