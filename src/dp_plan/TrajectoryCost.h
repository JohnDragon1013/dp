//
// Created by jydragon on 18-7-21.
//

#ifndef LATTICEPLAN_TRAJECTORYCOST_H
#define LATTICEPLAN_TRAJECTORYCOST_H

#include "ComparableCost.h"
#include "../QuinticPolynomialCurve1d.h"
#include "common.h"
#include "Obstacle.h"
#include "../CartesianFrenetConverter.h"
class TrajectoryCost {
public:
    TrajectoryCost() = default;

    TrajectoryCost(const common::DpPolyPathConfig &config,
                   const common::ReferenceLine reference_line,
                   const Obstacle obs,
                   const common::SLPoint &init_sl_point,
                   const common::FrenetFramePath lastSLpath
    );
    ComparableCost Calculate(const QuinticPolynomialCurve1d &curve,
                             const float start_s, const float end_s,
                             const uint32_t curr_level,
                             const uint32_t total_level);

private:
    ComparableCost CalculatePathCost(const QuinticPolynomialCurve1d &curve,
                                     const float start_s, const float end_s,
                                     const uint32_t curr_level,
                                     const uint32_t total_level);
    ComparableCost CalculateHistoricalCost(const QuinticPolynomialCurve1d &curve,
                                     const float start_s, const float end_s,
                                     const uint32_t curr_level,
                                     const uint32_t total_level);
    ComparableCost CalculateStaticObstacleCost(
            const QuinticPolynomialCurve1d &curve, const float start_s,
            const float end_s);
//    ComparableCost CalculateDynamicObstacleCost(
//            const QuinticPolynomialCurve1d &curve, const float start_s,
//            const float end_s) const;

    //FRIEND_TEST(AllTrajectoryTests, GetCostFromObsSL);
    ComparableCost GetCostFromObsSL(const float adc_s, const float adc_l);

    void setCarsize();

    const common::DpPolyPathConfig config_;
    const common::ReferenceLine reference_line_ ;
    bool is_change_lane_path_ = false;
    //const Car vehicle_param_;
    //SpeedData heuristic_speed_data_;
    const common::SLPoint init_sl_point_;
    uint32_t num_of_time_stamps_ = 0;
    //动态障碍物，目前用不着
    //std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes_;
    std::vector<float> obstacle_probabilities_;
    Obstacle m_obs;
    //静态障碍物 应该是sl坐标的边界值
    common::FrenetFramePath m_lastpath;
    Car Carsize[10];
};


#endif //LATTICEPLAN_TRAJECTORYCOST_H
