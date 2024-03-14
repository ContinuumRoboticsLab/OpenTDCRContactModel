//
// Created by Itai on 23/11/2023.
//

#include "pcca_solver_nlopt.h"
#include "pcca.h"

pcca_solver_nlopt::pcca_solver_nlopt() = default;

// TODO: check performance with classic pointer version instead of vectors
double nlopt_objective(unsigned n, const double* x, double* grad, void* f_data) {
    pcca* prob = (pcca*)f_data;
    if (grad != nullptr) {
        if (!prob->eval_grad_f(n, x, grad)) {
            throw std::runtime_error("Objective gradient evaluation failed.");
        }
    }
    double res;
    if (!prob->eval_f(n, x, res)) {
        throw std::runtime_error("Objective evaluation failed.");
    }
    return res;
}
struct constraint_data {
    pcca prob;
    int m;
    double* g; // contains constraint result (eq then ineq)
    double* grad;
    bool eq_called;
};

double nlopt_eq_constraint(unsigned n, const double* x, double* grad, void* c_data) {
    auto* d = (constraint_data*)c_data;
    d->eq_called=true;
    if (grad != nullptr) {
        if (!d->prob.eval_g(n, x, d->m, d->g, d->grad)) {
            throw std::runtime_error("Constraint value & gradient evaluation failed.");
        }
        // TODO: change pcca abi so it makes sense, then code here and other places wouldn't be terrible :(
        std::copy(d->grad, d->grad+n, grad);
    }
    else {
        if (!d->prob.eval_g(n, x, d->m, d->g, nullptr)) {
            // TODO: (follow up to upper TODO) this exception should be internal in eval_g instead of returning boolean...
            throw std::runtime_error("Constraint evaluation failed.");
        }
    }
    return d->g[0];
}
void nlopt_ineq_mconstraint(unsigned m, double *result, unsigned n, const double* x, double* grad, void* c_data) {
    // assuming eq constraint was already called
    auto* d = (constraint_data*)c_data;

    if (!d->eq_called)
        throw std::runtime_error("Inequality constraint called before eq constraint.");
    d->eq_called=false;

    if (grad != nullptr) {
        std::copy(d->grad+n, d->grad+(n*(d->m)), grad);
    }
    std::copy(d->g+1, d->g+m+1, result);
}

std::tuple<std::vector<double>, int>
pcca_solver_nlopt::solve(int n, double l, double l_tendon, double r, std::vector<double> x_init,
                         const xt::xtensor<double, 2> &tendon, const xt::xarray<double> &ab,
                         const xt::xarray<double> &s, const xt::xarray<double> &c_center) {
    int obstacles_num = (int) ab.shape()[1];
    int m = 1 + (n)*obstacles_num + 2*(2*n)*obstacles_num;;
    nlopt::opt opt(nlopt::LD_SLSQP, n);

    double g[m];
    double grad[n*m];
    struct constraint_data d {
        pcca(l/n, n, r, l_tendon, tendon, ab, s, c_center),
        m,
        g,
        grad
    };

    opt.set_min_objective(nlopt_objective, &d.prob);
    opt.add_equality_constraint(nlopt_eq_constraint, &d, 1e-10);
    opt.add_inequality_mconstraint(nlopt_ineq_mconstraint, &d, std::vector<double>(m-1, 1e-10));
    opt.set_ftol_rel(1e-11);
    opt.set_ftol_abs(1e-15);
    opt.set_xtol_rel(1e-10);
    opt.set_xtol_abs(1e-15);

    double min;
    try{
        nlopt::result result = opt.optimize(x_init, min);
        if (result == nlopt::result::MAXEVAL_REACHED || result == nlopt::result::MAXTIME_REACHED) {
            // I want all errors to be negative
            return {x_init, -result-2};
        }
        return {x_init, result};
    }
    catch(std::exception &e) {
        return {x_init, nlopt::result::FAILURE};
    }
}
