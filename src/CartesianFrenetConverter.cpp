//
// Created by jydragon on 18-7-23.
//

#include <cmath>
#include <iostream>
#include "CartesianFrenetConverter.h"
#include "dp_plan/Clothoid.h"
//程序输入包括 起点的(x,y) 弧长s
void CartesianFrenetConverter::cartesian_to_frenet(
        const double rs, //弧长
        const double rx, const double ry, const double rtheta,//参考点笛卡尔坐标下的(x,y,heading)
        const double rkappa, const double rdkappa, //参考点的k, dk
        const double x, const double y,//待转换点的(x,y)
        const double v, const double a, const double theta, const double kappa,//待转换点的其他信息
        //输出  s 三阶  d 三阶
        std::array<double, 3>* const ptr_s_condition,
        std::array<double, 3>* const ptr_d_condition)
{
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);
//判断角度差 判断距离的正负 正为左，负为右
    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;//法向（模）
    ptr_d_condition->at(0) =
            std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);//前面一个参数的值，后面一个参数的正负

    const double delta_theta = theta - rtheta;//角度差
    const double tan_delta_theta = std::tan(delta_theta);
    const double cos_delta_theta = std::cos(delta_theta);

    const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
    ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;

    const double kappa_r_d_prime =
            rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

    ptr_d_condition->at(2) =
            -kappa_r_d_prime * tan_delta_theta +
            one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
            (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

    ptr_s_condition->at(0) = rs;

    ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

    const double delta_theta_prime =
            one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
    ptr_s_condition->at(2) =
            (a * cos_delta_theta -
             ptr_s_condition->at(1) * ptr_s_condition->at(1) *
             (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
            one_minus_kappa_r_d;
    return;
}
//简化版的，认为两点平行，只算横向偏移
void CartesianFrenetConverter::cartesian_to_frenet(
        const double rs, const double rx, const double ry, const double rtheta,
        const double x, const double y, double* ptr_s, double* ptr_d) {
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;
    *ptr_d = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);
    *ptr_s = rs;
    return;
}

void CartesianFrenetConverter::frenet_to_cartesian(
        const double rs, const double rx, const double ry, const double rtheta,
        const double rkappa, const double rdkappa,
        const std::array<double, 3>& s_condition,
        const std::array<double, 3>& d_condition, double* const ptr_x,
        double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
        double* const ptr_v, double* const ptr_a) {
   // if(std::abs(rs - s_condition[0]) < 1.0e-6)
     //       std::cout<< "The reference point s and s_condition[0] don't match";

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    *ptr_x = rx - sin_theta_r * d_condition[0];
    *ptr_y = ry + cos_theta_r * d_condition[0];

    const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

    const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
    const double delta_theta = std::atan2(d_condition[1], one_minus_kappa_r_d);
    const double cos_delta_theta = std::cos(delta_theta);

    *ptr_theta = BasicStruct::AngleNormalnize1(delta_theta + rtheta);

    const double kappa_r_d_prime =
            rdkappa * d_condition[0] + rkappa * d_condition[1];
    *ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) *
                   cos_delta_theta * cos_delta_theta) /
                  (one_minus_kappa_r_d) +
                  rkappa) *
                 cos_delta_theta / (one_minus_kappa_r_d);

    const double d_dot = d_condition[1] * s_condition[1];
    *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d *
                       s_condition[1] * s_condition[1] +
                       d_dot * d_dot);

    const double delta_theta_prime =
            one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;

    *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
             s_condition[1] * s_condition[1] / cos_delta_theta *
             (d_condition[1] * delta_theta_prime - kappa_r_d_prime);
}

double CartesianFrenetConverter::CalculateTheta(const double rtheta,
                                                const double rkappa,
                                                const double l,
                                                const double dl) {
    return BasicStruct::AngleNormalnize1(rtheta + std::atan2(dl, 1 - l * rkappa));
}

double CartesianFrenetConverter::CalculateKappa(const double rkappa,
                                                const double rdkappa,
                                                const double l, const double dl,
                                                const double ddl) {
    double denominator = (dl * dl + (1 - l * rkappa) * (1 - l * rkappa));
    if (std::fabs(denominator) < 1e-8) {
        return 0.0;
    }
    denominator = std::pow(denominator, 1.5);
    const double numerator = rkappa + ddl - 2 * l * rkappa * rkappa -
                             l * ddl * rkappa + l * l * rkappa * rkappa * rkappa +
                             l * dl * rdkappa + 2 * dl * dl * rkappa;
    return numerator / denominator;
}

Vec2d CartesianFrenetConverter::CalculateCartesianPoint(const double rtheta,
                                                        const Vec2d& rpoint,
                                                        const double l) {
    const double x = rpoint.x - l * std::sin(rtheta);
    const double y = rpoint.y + l * std::cos(rtheta);
    return Vec2d(x, y);
}

double CartesianFrenetConverter::CalculateLateralDerivative(
        const double rtheta, const double theta, const double l,
        const double rkappa) {
    return (1 - rkappa * l) * std::tan(theta - rtheta);
}

double CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
        const double rtheta, const double theta, const double rkappa,
        const double kappa, const double rdkappa, const double l) {
    const double dl = CalculateLateralDerivative(rtheta, theta, l, rkappa);
    const double theta_diff = theta - rtheta;
    const double cos_theta_diff = std::cos(theta_diff);
    const double res = -(rdkappa * l + rkappa * dl) * std::tan(theta - rtheta) +
                       (1 - rkappa * l) / (cos_theta_diff * cos_theta_diff) *
                       (kappa * (1 - rkappa * l) / cos_theta_diff - rkappa);
    if (std::isinf(res)) {
        cout << "result is inf when calculate second order lateral "
                 "derivative. input values are rtheta:"
              << rtheta << " theta: " << theta << ", rkappa: " << rkappa
              << ", kappa: " << kappa << ", rdkappa: " << rdkappa << ", l: " << l
              << std::endl;
    }
    return res;
}
bool CartesianFrenetConverter::XY2SL(const PathPointxy referxy,common::ReferenceLine &referenceLine){
    if(referxy.pps.empty()) {
        cout<<("Reference lane is empty!")<<endl;
        return false;
    }
    double sum_s =0.0;
    double dis =10000;
    int index = -1;
    for (int i = 0; i < referxy.pps.size(); ++i) {
        double d =BasicStruct::Distance(g_currentLocation,referxy.pps[i]);
        double angle3 = atan2(referxy.pps[i].y-g_currentLocation.y,referxy.pps[i].x-g_currentLocation.x);
        double anglediff = cos(abs(angle3-g_currentLocation.angle));
        if(d<dis&&anglediff>0){
            dis = d;
            index = i;
        }
        if(i>20)
            break;
    }
    common::ReferencePoint lastp;// =referxy.pps[0];
    lastp.x=referxy.pps[index].x;
    lastp.y=referxy.pps[index].y;
    lastp.heading =referxy.pps[index].angle;
    lastp.s =0.0;
    lastp.l=0.0;
    lastp.kappa_=0.0;
    lastp.dkappa_=0.0;
    referenceLine.reference_points_.push_back(lastp);
    double k,dk,l;
    for(int rexy=index+1;rexy<referxy.pps.size();++rexy)
    {
        common::ReferencePoint reSL;
        //double dis =BasicStruct::Distance(referxy.pps[rexy],lastp);

        Clothoid::buildClothoid(lastp.x,lastp.y,lastp.heading,referxy.pps[rexy].x,referxy.pps[rexy].y,referxy.pps[rexy].angle,k,dk,l);
        sum_s+=l;
//        if(sum_s>10)
//            break;
        reSL.s =sum_s;
        reSL.l =0.0;
        reSL.x =referxy.pps[rexy].x;
        reSL.y =referxy.pps[rexy].y;
        reSL.heading = referxy.pps[rexy].angle;
        reSL.kappa_ =k;//referxy.pps[rexy].k;//(referxy.pps[rexy].angle-lastp.heading)/dis;
        reSL.dkappa_ =dk;//(reSL.kappa_-lastp.kappa_)/dis;
        lastp =reSL;//ferxy.pps[rexy];
        referenceLine.reference_points_.push_back(reSL);
    }
    referenceLine.Length =sum_s;
    if(referenceLine.reference_points_.empty())
        return false;
    return true;
}

common::TrajectoryPoint CartesianFrenetConverter::initial_Point_trans(common::ReferencePoint nearRefer, RoadPoint pointxy, double v,
                                                   double a, double kappa){
    //输出  s 三阶  d 三阶
    std::array<double, 3> ptr_s_condition;
    std::array<double, 3> ptr_d_condition;


    cartesian_to_frenet(nearRefer.s,nearRefer.x,nearRefer.y,nearRefer.heading,nearRefer.kappa_,nearRefer.dkappa_,
            pointxy.x,pointxy.y,v,a,pointxy.angle,kappa,&ptr_s_condition,&ptr_d_condition);
    common::TrajectoryPoint start;
    start.path_point.x=pointxy.x;
    start.path_point.y=pointxy.y;
    start.path_point.theta=pointxy.angle;
    start.path_point.s=ptr_s_condition.front();
    start.path_point.l=ptr_d_condition.front();
    start.v = v;
    start.a = a;
    return start;
}


