//
// Created by jydragon on 18-7-21.
//

#ifndef LATTICEPLAN_QUINTICPOLYNOMIALCURVE1D_H
#define LATTICEPLAN_QUINTICPOLYNOMIALCURVE1D_H


#include <array>

class QuinticPolynomialCurve1d {
public:
    QuinticPolynomialCurve1d() = default;

    QuinticPolynomialCurve1d(const std::array<double, 3>& start,
                             const std::array<double, 3>& end,
                             const double param);

    QuinticPolynomialCurve1d(const double x0, const double dx0, const double ddx0,
                             const double x1, const double dx1, const double ddx1,
                             const double param);

    QuinticPolynomialCurve1d(const QuinticPolynomialCurve1d& other);

    virtual ~QuinticPolynomialCurve1d() = default;

    double Evaluate(const std::uint32_t order, const double p) const ;

    double ParamLength() const { return param_; }
    std::string ToString() const ;
protected:
    void ComputeCoefficients(const double x0, const double dx0, const double ddx0,
                             const double x1, const double dx1, const double ddx1,
                             const double param);

    // f = sum(coef_[i] * x^i), i from 0 to 5
    std::array<double, 6> coef_{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 3> start_condition_{{0.0, 0.0, 0.0}};
    std::array<double, 3> end_condition_{{0.0, 0.0, 0.0}};
    double param_;
};



#endif //LATTICEPLAN_QUINTICPOLYNOMIALCURVE1D_H
