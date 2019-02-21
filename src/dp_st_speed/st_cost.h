//
// Created by jydragon on 18-8-1.
//

#ifndef LATTICEPLAN_ST_COST_H
#define LATTICEPLAN_ST_COST_H

#include <array>
#include <unordered_map>
#include "st_graph_point.h"
#include "../dp_plan/common.h"

class DpStCost {
public:
    DpStCost(const DpStSpeedConfig& dp_st_speed_config,
                      //const std::vector<const PathObstacle*>& obstacles,
                      const common::TrajectoryPoint& init_point);

    //float GetObstacleCost(const StGraphPoint& point);

    float GetReferenceCost(const STPoint& point,
                           const STPoint& reference_point) const;

    float GetSpeedCost(const STPoint& first, const STPoint& second,
                       const float speed_limit) const;

    float GetAccelCostByTwoPoints(const float pre_speed, const STPoint& first,
                                  const STPoint& second);
    float GetAccelCostByThreePoints(const STPoint& first, const STPoint& second,
                                    const STPoint& third);

    float GetJerkCostByTwoPoints(const float pre_speed, const float pre_acc,
                                 const STPoint& pre_point,
                                 const STPoint& curr_point);
    float GetJerkCostByThreePoints(const float first_speed,
                                   const STPoint& first_point,
                                   const STPoint& second_point,
                                   const STPoint& third_point);

    float GetJerkCostByFourPoints(const STPoint& first, const STPoint& second,
                                  const STPoint& third, const STPoint& fourth);

private:
    float GetAccelCost(const float accel);
    float JerkCost(const float jerk);

    //void AddToKeepClearRange(const std::vector<const PathObstacle*>& obstacles);
    static void SortAndMergeRange(
            std::vector<std::pair<float, float>>* keep_clear_range_);
    bool InKeepClearRange(float s) const;

    const DpStSpeedConfig& config_;
    //TODO:这里的obstacle部分怎么解决还是个问题 没有边界
    //const std::vector<const PathObstacle*>& obstacles_;
    const common::TrajectoryPoint& init_point_;

    float unit_t_ = 0.0;

    std::unordered_map<std::string, int> boundary_map_;
    std::vector<std::vector<std::pair<float, float>>> boundary_cost_;

    std::vector<std::pair<float, float>> keep_clear_range_;//存储各个障碍物的最小最大s，前一个为最小，后一个为最大

    std::array<float, 200> accel_cost_;
    std::array<float, 400> jerk_cost_;
};


#endif //LATTICEPLAN_ST_COST_H
