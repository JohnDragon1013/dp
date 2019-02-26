//
// Created by jydragon on 18-7-23.
//

#ifndef LATTICEPLAN_CARTESIANFRENETCONVERTER_H
#define LATTICEPLAN_CARTESIANFRENETCONVERTER_H


#include <array>
#include "BasicStruct.h"
struct Vec2d{
    double x;
    double y;
    Vec2d():x(0.0),y(0.0){};
    Vec2d(double x_,double y_):x(x_),y(y_){};
};
class CartesianFrenetConverter {
public:
    CartesianFrenetConverter() = delete;

    /*********************************************************
    ××××                                                 ××××
           这里用的stanford的论文，最后一页有完整的推导过程
    ××××                                                 ××××
    /*********************************************************/

    /**
   * Convert a vehicle state in Cartesian frame to Frenet frame.
   * Decouple a 2d movement to two independent 1d movement w.r.t. reference
   * line.
   * The lateral movement is a function of longitudinal accumulated distance s
   * to achieve better satisfaction of nonholonomic constraints.不完全约束
   */
    static void cartesian_to_frenet(const double rs, const double rx,
                                    const double ry, const double rtheta,
                                    const double rkappa, const double rdkappa,
                                    const double x, const double y,
                                    const double v, const double a,
                                    const double theta, const double kappa,
                                    std::array<double, 3>* const ptr_s_condition,
                                    std::array<double, 3>* const ptr_d_condition);

    static void cartesian_to_frenet(const double rs, const double rx,
                                    const double ry, const double rtheta,
                                    const double x, const double y, double* ptr_s,
                                    double* ptr_d);

    /**
     * Convert a vehicle state in Frenet frame to Cartesian frame.
     * Combine two independent 1d movement w.r.t. reference line to a 2d movement.
     */
    static void frenet_to_cartesian(const double rs, const double rx,
                                    const double ry, const double rtheta,
                                    const double rkappa, const double rdkappa,
                                    const std::array<double, 3>& s_condition,
                                    const std::array<double, 3>& d_condition,
                                    double* const ptr_x, double* const ptr_y,
                                    double* const ptr_theta,
                                    double* const ptr_kappa, double* const ptr_v,
                                    double* const ptr_a);

    // given sl point extract x, y, theta, kappa
    static double CalculateTheta(const double rtheta, const double rkappa,
                                 const double l, const double dl);

    static double CalculateKappa(const double rkappa, const double rdkappa,
                                 const double l, const double dl,
                                 const double ddl);

    static Vec2d CalculateCartesianPoint(const double rtheta, const Vec2d& rpoint,
                                         const double l);
    /**
     * @brief: given sl, theta, and road's theta, kappa, extract derivative l,
     *second order derivative l:
     */
    static double CalculateLateralDerivative(const double theta_ref,
                                             const double theta, const double l,
                                             const double kappa_ref);

    // given sl, theta, and road's theta, kappa, extract second order derivative
    static double CalculateSecondOrderLateralDerivative(
            const double theta_ref, const double theta, const double kappa_ref,
            const double kappa, const double dkappa_ref, const double l);
};


#endif //LATTICEPLAN_CARTESIANFRENETCONVERTER_H
