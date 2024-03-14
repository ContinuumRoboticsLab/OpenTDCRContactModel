//
// Created by Itai on 11/11/2023.
//
#include "model_utils.h"
#include "xtensor-blas/xlinalg.hpp"

xt::xtensor<double,2> model_utils::trans_mat(double k, double l) {
    auto theta = k*l;
    double phi = 0;
    double epsi = 0;

    double T11 = cos(phi)*cos(epsi-phi)*cos(theta)-sin(phi)*sin(-phi+epsi);
    double T12 = -cos(phi)*sin(-phi+epsi)*cos(theta)-sin(phi)*cos(-phi+epsi);
    double T13 = cos(phi)*sin(theta);
    double T14 = (1-cos(theta))*cos(phi)/k;
    double T21 = sin(phi)*cos(epsi-phi)*cos(theta)+cos(phi)*sin(-phi+epsi);
    double T22 = cos(phi)*cos(-phi+epsi)-sin(phi)*sin(-phi+epsi)*cos(theta);
    double T23 = sin(phi)*sin(theta);
    double T24 = (1-cos(theta))*sin(phi)/k;
    double T31 = -cos(-phi+epsi)*sin(theta);
    double T32 = sin(-phi+epsi)*sin(theta);
    double T33 = cos(theta);
    double T34 = sin(theta)/k;
    double T41 = 0;
    double T42 = 0;
    double T43 = 0;
    double T44 = 1;

    return xt::xtensor<double,2> {{T11, T12, T13, T14},
                               {T21, T22, T23, T24},
                               {T31, T32, T33, T34},
                               {T41, T42, T43, T44}};

}

xt::xtensor<double,2> model_utils::trans_mat_derivative(double k, double l) {
    auto theta = k*l;
    double phi = 0;
    double epsi = 0;

    double T11 = cos(phi)*cos(epsi-phi)*(-sin(theta)*l); //
    double T12 = -cos(phi)*sin(-phi+epsi)*(-sin(theta)*l); //
    double T13 = cos(phi)*(cos(theta)*l); //
    double T14 = (-(-sin(theta)*l))*cos(phi)/k - (1-cos(theta))*cos(phi)/(k*k); //
    double T21 = sin(phi)*cos(epsi-phi)*(-sin(theta)*l); //
    double T22 = -sin(phi)*sin(-phi+epsi)*(-sin(theta)*l); //
    double T23 = sin(phi)*(cos(theta)*l); //
    double T24 = (-(-sin(theta)*l))*sin(phi)/k - (1-cos(theta))*sin(phi)/(k*k); //
    double T31 = -cos(-phi+epsi)*(cos(theta)*l); //
    double T32 = sin(-phi+epsi)*(cos(theta)*l); //
    double T33 = (-sin(theta)*l); //
    double T34 = (cos(theta)*l)/k - sin(theta)/(k*k); //
    double T41 = 0;
    double T42 = 0;
    double T43 = 0;
    double T44 = 0;

    return xt::xtensor<double,2> {{T11, T12, T13, T14},
                                  {T21, T22, T23, T24},
                                  {T31, T32, T33, T34},
                                  {T41, T42, T43, T44}};
}
