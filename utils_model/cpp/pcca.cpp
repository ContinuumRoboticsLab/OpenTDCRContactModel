//
// Created by Itai on 09/11/2023.
//

#include <algorithm>
#include <numbers>

#include "xtensor-blas/xlinalg.hpp"
#include "xtensor/xmath.hpp"
#include "xtensor/xadapt.hpp"
#include "xtensor/xindex_view.hpp"

#include "pcca.h"
#include "model_utils.h"

bool pcca::eval_f(int n, const double *x, double &obj_value) const {
    for (auto i = 0; i < n; i++) { obj_value += (l_inter * x[i]) * (l_inter * x[i]); }
    return true;
}

bool pcca::eval_grad_f(int n, const double *x, double *grad_f) const {
    for (auto i = 0; i < n; i++) { grad_f[i] = 2 * x[i] * l_inter * l_inter; }
    return true;
}
bool pcca::eval_g(int n, const double *x, int m, double *g, double *g_derivative) const {
    xt::xarray<double> pt = p_tendon;
    xt::xtensor<double,4> p = xt::zeros<double>({4, 5*n, 1, 1});
    auto size = ((int)ab.shape()[2]) * 5 * n;
    auto g_tensor = xt::adapt(g+1, size, xt::no_ownership(), std::vector{size});

    xt::xtensor<double,2> T = xt::eye(4);
    xt::xtensor<double,1> lt = xt::zeros<double>({2});

    // derivative
    xt::xtensor<double,3> pt_derivative;
    xt::xtensor<double,4> p_derivative;
    // TODO: find a way to split declaration and initialization
    auto g_derivative_tensor = g_tensor; // just a hack to get the declaration of g_derivative_tensor right using "auto".
    // in numpy can broadcast with matmul instead of using a for loop, might be very significant
    // maybe other c++ libraries can do the same
    std::vector<xt::xtensor<double,2>> dT;
    xt::xtensor<double,2> lt_derivative;
    if (g_derivative) {
        pt_derivative = xt::zeros<double>({4,2, n});
        p_derivative = xt::zeros<double>({4, 5*n, 1, n});
        g_derivative_tensor = xt::adapt(g_derivative + n, (m - 1) * n, xt::no_ownership(), std::vector{(m - 1) * n});
        dT = std::vector<xt::xtensor<double,2>>(n, xt::eye(4));
        lt_derivative = xt::zeros<double>({2,n});
    }

    for (int i = 0; i < n; i++) {
        auto k = x[i];
        xt::xtensor<double,2> T_local = model_utils::trans_mat(k, l_inter);
        T = xt::linalg::dot(T, T_local);

        xt::view(p, xt::all(), i*5, 0, 0) = xt::view(T, xt::all(), 3);
        auto pt_new = xt::linalg::dot(T, p_tendon);
        xt::xarray<double> lt_seg = xt::sqrt(xt::sum(xt::pow(pt_new - pt, 2), 0));
        lt += lt_seg;

        auto pt_mid = (pt_new + pt) / 2;
        xt::view(p, xt::all(), xt::range(i*5+1, i*5+3), 0, 0) = pt_new;
        xt::view(p, xt::all(), xt::range(i*5+3, i*5+5), 0, 0) = pt_mid;

        if (g_derivative) {
            xt::xtensor<double,2> dT_local = model_utils::trans_mat_derivative(k, l_inter);
            xt::xtensor<double,3> pt_new_derivative = xt::zeros<double>({4,2,n});
            for (int j = 0; j < n; j++) {
                if (j < i || j > i) {
                    dT[j] = xt::linalg::dot(dT[j], T_local);
                }
                else {
                    dT[j] = xt::linalg::dot(dT[j], dT_local);
                }

                if (j <= i) {
                    xt::view(p_derivative, xt::all(), i*5, 0, j) = xt::view(dT[j], xt::all(), 3);
                    xt::view(pt_new_derivative, xt::all(), xt::all(), j) = xt::linalg::dot(dT[j], p_tendon);
                }
            }

            xt::xtensor<double,3> pt_mid_derivative = (pt_new_derivative + pt_derivative) / 2;

            xt::view(p_derivative, xt::all(), xt::range(i*5+1, i*5+3), 0, xt::all()) = pt_new_derivative;
            xt::view(p_derivative, xt::all(), xt::range(i*5+3, i*5+5), 0, xt::all()) = pt_mid_derivative;

            auto temp11= (1/lt_seg.reshape({2,1}));
            auto temp21 = pt_new.reshape({4,2,1});
            auto temp22 = pt.reshape({4,2,1});
            auto temp31 = pt_new_derivative-pt_derivative;
            auto temp32 = temp21-temp22;
            auto temp12 = xt::sum(temp32*temp31,{0});
            auto temp13 =  temp11 * temp12;
            xt::view(lt_derivative, xt::all(), xt::all()) += temp13;

            pt_derivative = pt_new_derivative;

            pt = pt_new.reshape({4,2});
        }
        else {
            pt = pt_new;
        }
    }

    auto temp1 = ((p-c_center) / ab);
    auto temp2 = xt::pow(temp1,s);
    auto temp3 = 1 - xt::sum(temp2,0);
    g_tensor = xt::flatten(temp3);

    xt::xtensor<bool,1> cond = g_tensor < -1;
    xt::filter(g_tensor, cond) = -1;
    g[0] = input_tendon-lt(0);

    if (g_derivative) {
        auto temp5 = s * xt::pow(temp1, s - 1) * (p_derivative / ab);

        auto temp6 = -xt::sum(temp5, 0);
        g_derivative_tensor = xt::flatten(temp6);
        auto cond_repeat = xt::repeat(cond, n, 0);
        xt::filter(g_derivative_tensor, cond_repeat) = 0;

        auto lt_derivative_0 = (-xt::view(lt_derivative, 0, xt::all()));
        std::copy(lt_derivative_0.begin(), lt_derivative_0.end(), g_derivative);
    }

    return true;
}

bool pcca::struct_jac_g(int n, int m, int nele_jac, int *iRow, int *jCol) const {
    for (auto i = 0; i < m; i++) {
        for (auto j = 0; j < n; j++) {
            iRow[i*n+j] = i;
            jCol[i*n+j] = j;
        }
    }
    return true;
}

bool pcca::eval_jac_g(int n, int m, const double *x, int nele_jac, double *values) const {
    double g_mh[m];
    double g_ph[m];
    double h = 1e-8;
//    double y[n];
//    std::copy(x, x+n, y);
    auto *y = const_cast<double*>(x);
    for (int i = 0; i<n; i++) {
        y[i] -= h;
        bool eval_mh = eval_g(n, y, m, g_mh, nullptr);
        if (!eval_mh) return false;
        y[i] += 2*h;
        bool eval_ph = eval_g(n, y, m, g_ph, nullptr);
        y[i] -= h;
        if (!eval_ph) return false;
        for (int j = 0; j < m; j++) {
            values[j*n+i] = (g_ph[j] - g_mh[j]) / (2 * h);
        }
    }

        return true;
}

bool pcca::struct_h(int n, int m, int nele_hess, int *iRow, int *jCol) const {
    // should use hessian approx in app options
    return false;
}

bool pcca::eval_h(int n, const double *x, double obj_factor, int m, const double *lambda, int nele_hess,
                  double *values) const {
    // should use hessian approx in app options
    return false;
}

pcca::pcca(double l_inter, int n_disk, double r_disk, double input_tendon,
           const xt::xtensor<double,2> &p_tendon,
           const xt::xarray<double> &ab, const xt::xarray<double> &s,
           const xt::xarray<double> &c_center) : l_inter(l_inter), n_disk(n_disk), r_disk(r_disk),
                                                             input_tendon(input_tendon), p_tendon(p_tendon) {
    if (l_inter <= 0 || n_disk <= 0 || r_disk <= 0 || input_tendon <= 0) {
        throw std::invalid_argument("Received non-positive value(s).");
    }
    this->ab = xt::adapt(ab.data(), std::vector<int>{4,1,(int)ab.shape(1),1});
    this->s = xt::adapt(s.data(), std::vector<int>{1,1,(int)ab.shape(1),1});
    this->c_center = xt::adapt(c_center.data(), std::vector<int>{4,1,(int)ab.shape(1),1});
}
