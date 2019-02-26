//
// Created by jydragon on 18-8-7.
//

#ifndef LATTICEPLAN_ST_BOUNDARY_H
#define LATTICEPLAN_ST_BOUNDARY_H


#include <limits>
#include <string>
#include "SpeedData.h"
#include "../BasicStruct.h"

struct StBoundaryConfig {
        double boundary_buffer = 0.1 ;
        double high_speed_centric_acceleration_limit = 0.8;//1.2 ;
        double low_speed_centric_acceleration_limit = 1.2;//1.4 ;
        double high_speed_threshold = 12.58;//20.0 ;
        double low_speed_threshold = 7.0 ;
        double minimal_kappa = 0.00001 ;
        double point_extension = 1.0 ;
        double lowest_speed = 2.5 ;
        int num_points_to_avg_kappa = 2;//4 ;
        double static_obs_nudge_speed_ratio = 0.6;
        double dynamic_obs_nudge_speed_ratio = 0.8;
        double centri_jerk_speed_coeff = 1.0;
};
struct LineSegment2d{
    RoadPoint start;
    RoadPoint end;
    RoadPoint unit_direction;
    double heading = 0.0;
    double length = 0.0;
};

class StBoundary {
public:
    StBoundary ()= default;
    StBoundary(const std::vector<std::pair<STPoint, STPoint>>& point_pairs); //前一个点是左下，后一个点是右上
    //StBoundary(std::vector<common::math::Vec2d> points) = delete;
    ~StBoundary() = default;
    bool IsEmpty() const { return lower_points_.empty(); }
    bool IsPointInBoundary(const STPoint& st_point) const;
    bool HasOverlap(const STPoint& p1, const STPoint& p2)const;


    // if you need to add boundary type, make sure you modify
    // GetUnblockSRange accordingly.
    enum class BoundaryType {
        UNKNOWN,
        STOP,
        FOLLOW,
        YIELD,//避让
        OVERTAKE,
        KEEP_CLEAR,
    };
    //static std::string TypeName(BoundaryType type);

    BoundaryType boundary_type() const;
    void SetBoundaryType(const BoundaryType& boundary_type);


    double min_s() const;
    double min_t() const;
    double max_s() const;
    double max_t() const;

    void setmin_s(double xx) {min_s_ = xx;}
    void setmin_t(double xx) {min_t_ = xx;}
    void setmax_s(double xx) {max_s_ = xx;}
    void setmax_t(double xx) {max_t_ = xx;}
    double Area() const;

    std::vector<STPoint> upper_points() const { return upper_points_; }
    std::vector<STPoint> lower_points() const { return lower_points_; }

private:
    BoundaryType boundary_type_ = BoundaryType::UNKNOWN;

    std::vector<STPoint> upper_points_;
    std::vector<STPoint> lower_points_;

    std::string id_;
    double characteristic_length_ = 1.0;
    double s_high_limit_ = 200.0;
    double min_s_ = std::numeric_limits<double>::max();
    double max_s_ = std::numeric_limits<double>::lowest();
    double min_t_ = std::numeric_limits<double>::max();
    double max_t_ = std::numeric_limits<double>::lowest();


    void BuildFromPoints();
    int Next(int at) const;
    int Prev(int at) const;

    //static bool ClipConvexHull(const LineSegment2d &line_segment, std::vector<Vec2d> *const points);

    std::vector<STPoint> points_;
    int num_points_ = 0;
//    std::vector<LineSegment2d> line_segments_;
    bool is_convex_ = false;
    double area_ = 0.0;
//    double min_x_ = 0.0;
//    double max_x_ = 0.0;
//    double min_y_ = 0.0;
//    double max_y_ = 0.0;
};


#endif //LATTICEPLAN_ST_BOUNDARY_H
