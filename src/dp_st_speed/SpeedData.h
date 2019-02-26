//
// Created by jydragon on 18-7-31.
//

#ifndef LATTICEPLAN_SPEEDDATA_H
#define LATTICEPLAN_SPEEDDATA_H

#include <vector>
#include <limits>
struct DpStSpeedConfig {
    double total_path_length = 79;//80
    double total_time = 19.0;
    int matrix_dimension_s = 80 ;//80
    int matrix_dimension_t = 20; //

    double speed_weight = 0.0;
    double accel_weight = 10.0;
    double jerk_weight = 10.0;
    double obstacle_weight = 1.0;
    double reference_weight = 0.0;
    double go_down_buffer = 5.0;
    double go_up_buffer = 5.0;

      // obstacle cost config
    double default_obstacle_cost = 1e3;

       // speed cost config
    double default_speed_cost = 1.0e3;
    double exceed_speed_penalty = 10.0; //超速
    double low_speed_penalty = 10.0;//2.5 //低于限速
    double keep_clear_low_speed_penalty = 10.0;

       // accel cost config
    double accel_penalty = 1.0;
    double decel_penalty = 1.0;

    // jerk cost config
    double positive_jerk_coeff = 1.0;
    double negative_jerk_coeff = 1.0;//300.0;

    // other constraint
    double max_acceleration = 3.0;//[ default = 4.5 ];
    double max_deceleration = -4.0;// [ default = -4.5 ];

    //apollo.planning.StBoundaryConfig st_boundary_config = 50;

    struct st_boundary_config {
        double boundary_buffer= 0.1;
        float high_speed_centric_acceleration_limit= 0.8;
        float low_speed_centric_acceleration_limit= 1.2;
        float high_speed_threshold= 12.58;
        float low_speed_threshold= 7.5;
        double minimal_kappa= 0.00001;
        float point_extension= 1.0;
        float lowest_speed = 2.5;
        float num_points_to_avg_kappa = 2;
        float static_obs_nudge_speed_ratio = 0.6;
        float dynamic_obs_nudge_speed_ratio = 0.8;
        float centri_jerk_speed_coeff = 1.0;
    };
};

struct STPoint{
    STPoint() = default;
    STPoint(const double s_, const double t_):s(s_),t(t_){};
    //explicit STPoint(const common::math::Vec2d& vec2d_point);

//    double s() const;
//    double t() const;
//    void set_s(const double s);
//    void set_t(const double t);
    double s=0.0;
    double t=0.0;
};
struct SpeedPoint{
    SpeedPoint ()= default;
    SpeedPoint(const double _s, const double _t, const double _v,
               const double _a, const double _da):s(_s),t(_t),v(_v),a(_a),da(_da){};
    ~SpeedPoint()= default;
    double s ;
    double t ;
    // speed (m/s)
    double v ;
    // acceleration (m/s^2)
    double a ;
    // jerk (m/s^3)
    double da ;
    void set_s(const double _s){s = _s;};
    void set_t(const double _t){t = _t;};
    void set_v(const double _v){v = _v;};
//    void set_t(const double _t){t = _t;}
};

struct SLBoundary {
    double start_s = std::numeric_limits<double >::max();
    double end_s =-99999999;//std::numeric_limits<double >::min();
    double start_l =std::numeric_limits<double >::max();
    double end_l =-99999999;//std::numeric_limits<double >::min();
};
class SpeedData {
public:
    SpeedData() = default;

    SpeedData(std::vector<SpeedPoint> speed_points);

    ~SpeedData() = default;

    const std::vector<SpeedPoint>& speed_vector() const;

    void set_speed_vector(std::vector<SpeedPoint> speed_points);

    void AppendSpeedPoint(const double s, const double time, const double v,
                          const double a, const double da);

    bool EvaluateByTime(const double time,
                        SpeedPoint* const speed_point) const;

    double TotalTime() const;

    bool Empty() const { return speed_vector_.empty(); }

    void Clear();

    //virtual std::string DebugString() const;

private:
    std::vector<SpeedPoint> speed_vector_;
};

#endif //LATTICEPLAN_SPEEDDATA_H
