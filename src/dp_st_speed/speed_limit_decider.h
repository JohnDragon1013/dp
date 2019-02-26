//
// Created by jydragon on 18-8-7.
//

#ifndef LATTICEPLAN_SPEED_LIMIT_DECIDER_H
#define LATTICEPLAN_SPEED_LIMIT_DECIDER_H


#include "SpeedData.h"
#include "../dp_plan/pathdata.h"
#include "speed_limit.h"
#include "st_path_obstacle_test.h"

class SpeedLimitDecider {
public:
    SpeedLimitDecider(const StBoundaryConfig& config,
//                      const common::ReferenceLine& reference_line,
                      const PathData& path_data);

    ~SpeedLimitDecider() = default;

    bool GetSpeedLimits(
            const vector< PathObstacle>& path_obstacles,
            SpeedLimit* const speed_limit_data) const;

private:
    double GetCentricAccLimit(const double kappa) const;

    void GetAvgKappa(const std::vector<common::PathPoint>& path_points,
                     std::vector<double>* kappa) const;

private:
//    const SLBoundary& adc_sl_boundary_;
    const StBoundaryConfig& st_boundary_config_;
//    const common::ReferenceLine& reference_line_;
    const PathData& path_data_;
//    const common::VehicleParam& vehicle_param_;
};


#endif //LATTICEPLAN_SPEED_LIMIT_DECIDER_H
