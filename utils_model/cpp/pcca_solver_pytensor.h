//
// Created by Itai on 21/11/2023.
//

#ifndef CONTACT_MODEL_CPP_PCCA_SOLVER_PYTENSOR_H
#define CONTACT_MODEL_CPP_PCCA_SOLVER_PYTENSOR_H

#include "pcca_solver.h"
#include <xtensor-python/pytensor.hpp>
#include <xtensor-python/pyarray.hpp>

class pcca_solver_pytensor {
public:
    pcca_solver_pytensor();

    /**
     *
     * @param n number of sections
     * @param l length of backbone
     * @param l_tendon length of tendon
     * @param r radius of robot
     * @param x_init initial guess for the solution
     * @param tendon a (4,2) array for the first and second tendon vectors. Usually [(r,0,0,1),(-r,0,0,1)]. Last value in the first dimension should always be 1.
     * @param ab a (4,number_of_obstacles) array, containing a vector (first dimension) for every obstacle (second dimension). The vector contains (a,b,c,1) for the superellipse.
     * @param s a (number_of_obstacles) array, containing the power for the superellipse of each obstacles.
     * @param c_center a (4,number_of_obstacles) array for the centers (x0,y0,z0,1) of each obstacle.
     * @return proposed solution and exit code. Exit codes are different for different algorithms. Negative exit codes mean failure and the solution should be ignored.
     */
    std::tuple<std::vector<double>, int> solve(int n, double l, double l_tendon, double r, std::vector<double> x_init,
                                               const xt::pytensor<double, 2>& tendon, const xt::pyarray<double>& ab,
                                               const xt::pyarray<double>& s, const xt::pyarray<double>& c_center);
private:
    std::unique_ptr<pcca_solver> solver;
};


#endif //CONTACT_MODEL_CPP_PCCA_SOLVER_PYTENSOR_H
