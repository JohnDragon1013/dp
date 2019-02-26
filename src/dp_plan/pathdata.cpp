//
// Created by jydragon on 18-8-7.
//
#include <algorithm>
#include <iostream>
#include "pathdata.h"
bool PathData::SetDiscretizedPath(const common::DiscretizedPath &path) {
//    if (reference_line_ == nullptr) {
//        cout << "Should NOT set discretized path when reference line is nullptr. "
//                  "Please set reference line first.";
//        return false;
//    }
    discretized_path_ = path;
//    if (!XYToSL(discretized_path_, &frenet_path_)) {
//        cout << "Fail to transfer discretized path to frenet path.";
//        return false;
//    }
//    DCHECK_EQ(discretized_path_.NumOfPoints(), frenet_path_.points().size());
    path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
    return true;
}

bool PathData::SetFrenetPath(const common::FrenetFramePath &frenet_path) {
//    if (reference_line_ == nullptr) {
//        cout << "Should NOT set frenet path when reference line is nullptr. "
//                  "Please set reference line first.";
//        return false;
//    }
    frenet_path_ = frenet_path;
//    if (!SLToXY(frenet_path_, &discretized_path_)) {
//        cout << "Fail to transfer frenet path to discretized path.";
//        return false;
//    }
    //DCHECK_EQ(discretized_path_.NumOfPoints(), frenet_path_.points().size());
    path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
    return true;
}

const common::DiscretizedPath &PathData::discretized_path() const {
    return discretized_path_;
}

bool PathData::Empty() const {
    return discretized_path_.size() == 0 &&
           frenet_path_.empty();
}

std::list<std::pair<common::DiscretizedPath, common::FrenetFramePath>>
&PathData::path_data_history() {
    return path_data_history_;
}

const common::FrenetFramePath &PathData::frenet_frame_path() const {
    return frenet_path_;
}

void PathData::SetReferenceLine(const common::ReferenceLine *reference_line) {
    Clear();
    reference_line_ = reference_line;
}

//bool PathData::GetPathPointWithPathS(
//        const double s, common::PathPoint *const path_point) const {
//    *path_point = discretized_path_.Evaluate(s);
//    return true;
//}

bool PathData::GetPathPointWithRefS(const double ref_s,
                                    common::PathPoint *const path_point) const {
//    DCHECK_NOTNULL(reference_line_);
//    DCHECK_NOTNULL(path_point);
//    DCHECK_EQ(discretized_path_.path_points().size(),
//              frenet_path_.points().size());
//    if (ref_s < 0) {
//        cout << "ref_s[" << ref_s << "] should be > 0";
//        return false;
//    }
//    if (ref_s > frenet_path_.back().s) {
//        cout << "ref_s is larger than the length of frenet_path_ length ["
//               << frenet_path_.back().s << "].";
//        return false;
//    }
//
//    uint32_t index = 0;
//    const double kDistanceEpsilon = 1e-3;
//    for (uint32_t i = 0; i + 1 < frenet_path_.points().size(); ++i) {
//        if (fabs(ref_s - frenet_path_.points().at(i).s()) < kDistanceEpsilon) {
//            path_point->CopyFrom(discretized_path_.pps.at(i));
//            return true;
//        }
//        if (frenet_path_.points().at(i).s() < ref_s &&
//            ref_s <= frenet_path_.points().at(i + 1).s()) {
//            index = i;
//            break;
//        }
//    }
//    double r = (ref_s - frenet_path_.points().at(index).s()) /
//               (frenet_path_.points().at(index + 1).s() -
//                frenet_path_.points().at(index).s());
//
//    const double discretized_path_s =
//            discretized_path_.path_points().at(index).s() +
//            r * (discretized_path_.path_points().at(index + 1).s() -
//                 discretized_path_.path_points().at(index).s());
//    path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));

    return true;
}

void PathData::Clear() {
    discretized_path_.clear();
    frenet_path_.clear();
    reference_line_ = nullptr;
}
std::vector<common::PathPoint>::const_iterator QueryLowerBound(
        common::DiscretizedPath path_points_,
        const double path_s)
{
    auto func = [](const common::PathPoint &tp, const double path_s) {
        return tp.s < path_s;
    };
    return std::lower_bound(path_points_.begin(), path_points_.end(), path_s,func);
}
double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
    double kMathEpsilon = 1e-10;
    if (std::abs(t1 - t0) <= kMathEpsilon) {
        cout << "input time difference is too small"<<endl;
        return BasicStruct::AngleNormalnize1(a0);
    }
    const double a0_n = BasicStruct::AngleNormalnize1(a0);
    const double a1_n = BasicStruct::AngleNormalnize1(a1);
    double d = a1_n - a0_n;
    if (d > M_PI) {
        d = d - 2 * M_PI;
    } else if (d < -M_PI) {
        d = d + 2 * M_PI;
    }

    const double r = (t - t0) / (t1 - t0);
    const double a = a0_n + d * r;
    return BasicStruct::AngleNormalnize1(a);
}
//线性逼近
common::PathPoint InterpolateUsingLinearApproximation(const common::PathPoint &p0,
                                              const common::PathPoint &p1,
                                              const double s) {
    double s0 = p0.s;
    double s1 = p1.s;

    common::PathPoint path_point;
    double weight = (s - s0) / (s1 - s0);
    double x = (1 - weight) * p0.x + weight * p1.x;
    double y = (1 - weight) * p0.y + weight * p1.y;
    double theta = slerp(p0.theta, p0.s, p1.theta, p1.s, s);
    double kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
    double dkappa = (1 - weight) * p0.dkappa + weight * p1.dkappa;
    double ddkappa = (1 - weight) * p0.ddkappa + weight * p1.ddkappa;
    path_point.x = (x);
    path_point.y = (y);
    path_point.theta = (theta);
    path_point.kappa = (kappa);
    path_point.dkappa = (dkappa);
    path_point.ddkappa = (ddkappa);
    path_point.s = (s);
    return path_point;
}
bool PathData::GetPathPointWithPathS(const double s, common::PathPoint *const path_point) const {
//    *path_point = discretized_path_.Evaluate(s);

    auto it_lower = QueryLowerBound(discretized_path_,s);
    if (it_lower == discretized_path_.begin()) {
        *path_point = discretized_path_.front();
        return 1;
    }
    if (it_lower == discretized_path_.end()) {
        *path_point = discretized_path_.back();
        return 1;
    }
    *path_point = InterpolateUsingLinearApproximation(*(it_lower - 1),
                                                             *it_lower, s);
    return true;
}

