//
// Created by Itai on 09/11/2023.
//

#ifndef CONTACT_MODEL_CPP2_PCCA_H
#define CONTACT_MODEL_CPP2_PCCA_H

#include <vector>

#include "xtensor/xarray.hpp"
#include "xtensor/xtensor.hpp"

class pcca {
public:
    pcca(double l_inter, int n_disk, double r_disk, double input_tendon,
         const xt::xtensor<double,2> &p_tendon,
         const xt::xarray<double> &ab, const xt::xarray<double> &s,
         const xt::xarray<double> &c_center);

    /** Method to return the objective value */
    virtual bool eval_f(
            int         n,
            const double* x,
            double&       obj_value
    ) const;

    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(
            int         n,
            const double* x,
            double*       grad_f
    ) const;

    /** Method to return the constraint residuals */
    virtual bool eval_g(
            int         n,
            const double* x,
            int         m,
            double*       g,
            double*       g_derivative
    ) const;

    /** Method to return the structure of the jacobian */
    virtual bool struct_jac_g(
            int         n,
            int         m,
            int         nele_jac,
            int*        iRow,
            int*        jCol
    ) const;

    /** Method to return the values of the jacobian */
    virtual bool eval_jac_g(
            int         n,
            int         m,
            const double* x,
            int         nele_jac,
            double*       values
    ) const;

    /** Method to return the structure of the hessian of the lagrangian (if "values" is NULL) */
    virtual bool struct_h(
            int         n,
            int         m,
            int         nele_hess,
            int*        iRow,
            int*        jCol
    ) const;

    /** Method to return the values of the hessian of the lagrangian (if "values" is not NULL) */
    virtual bool eval_h(
            int         n,
            const double* x,
            double        obj_factor,
            int         m,
            const double* lambda,
            int         nele_hess,
            double*       values
    ) const;
private:
    double l_inter;
    int n_disk;
    double r_disk;
    double input_tendon;
    xt::xtensor<double,2> p_tendon;
    xt::xtensor<double,4> ab;
    xt::xtensor<double,4> s;
    xt::xtensor<double,4> c_center;
};


#endif //CONTACT_MODEL_CPP2_PCCA_H
