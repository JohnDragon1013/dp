//
// Created by jydragon on 18-8-7.
//

#include <cmath>
#include "st_boundary.h"
#include "../BasicStruct.h"

StBoundary::StBoundary(
        const std::vector<std::pair<STPoint, STPoint>>& point_pairs) {

    std::vector<std::pair<STPoint, STPoint>> reduced_pairs(point_pairs);
//    RemoveRedundantPoints(&reduced_pairs);

    for (const auto& item : reduced_pairs) {
        // use same t for both points
        const double t = item.first.t;
        lower_points_.emplace_back(item.first.s, item.first.t);//同一时刻的不同s 即障碍物长度
        upper_points_.emplace_back(item.second.s, item.second.t);
    }
//
//    for (auto it = lower_points_.begin(); it != lower_points_.end(); ++it) {
//        points_.emplace_back(it->s, it->t);
//    }
//    for (auto rit = upper_points_.rbegin(); rit != upper_points_.rend(); ++rit) {
//        points_.emplace_back(rit->s, rit->t);
//    }
//
//    //BuildFromPoints();
//
//    for (const auto& point : lower_points_) {
//        min_s_ = std::fmin(min_s_, point.s);
//    }
//    for (const auto& point : upper_points_) {
//        max_s_ = std::fmax(max_s_, point.s);
//    }
    min_s_ = point_pairs[0].first.s;
    max_s_ = point_pairs[0].second.s;
    min_t_ = point_pairs[0].first.t;
    max_t_ = point_pairs[0].second.t;
}
StBoundary::BoundaryType StBoundary::boundary_type() const { return boundary_type_;}
double StBoundary::min_s() const { return min_s_; }
double StBoundary::min_t() const { return min_t_; }
double StBoundary::max_s() const { return max_s_; }
double StBoundary::max_t() const { return max_t_; }
void StBoundary::SetBoundaryType(const BoundaryType& boundary_type) {
    boundary_type_ = boundary_type;
}

bool StBoundary::IsPointInBoundary(const STPoint &st_point) const {
    if(st_point.s >= min_s_ && st_point.s <= max_s_ &&
        st_point.t >= min_t_ && st_point.t <= max_t_)
        return true;
    return false;
}

bool StBoundary::HasOverlap(const STPoint &p1, const STPoint &p2) const {
//    LineSegment2d line_segment;
//    line_segment.start = RoadPoint(p1.s,p1.t,0,0);
//    line_segment.end  = RoadPoint(p2.s,p2.t,0,0);
////    line_segment.length =std::sqrt( p1.s-p2.s   );
//    //out of range
//    if ((line_segment.start.x < min_t() && line_segment.end.x < min_t()) ||
//        (line_segment.start.x > max_t() && line_segment.end.x > max_t()) ||
//        (line_segment.start.y < min_s() && line_segment.end.y < min_s()) ||
//        (line_segment.start.y > max_s() && line_segment.end.y > max_s())) {
//        return false;
//    }
    vector<double> vx;
    vx.push_back(min_t());
    vx.push_back(min_t());
    vx.push_back(max_t());
    vx.push_back(max_t());
    vector<double> vy;
    vy.push_back(min_s());
    vy.push_back(max_s());
    vy.push_back(max_s());
    vy.push_back(min_s());
    return BasicStruct::LineSegOverPolygon(vx,vy,p1.t,p1.s,p2.t,p2.s);
}
