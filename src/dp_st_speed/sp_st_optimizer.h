//
// Created by jydragon on 18-8-4.
//

#ifndef LATTICEPLAN_SP_ST_OPTIMIZER_H
#define LATTICEPLAN_SP_ST_OPTIMIZER_H


#include "SpeedData.h"
#include "../dp_plan/common.h"
#include "../dp_plan/pathdata.h"
#include "st_boundary.h"
#include "speed_limit.h"
//#include "st_graph_data.h"
#include "speed_limit_decider.h"

class DpStSpeedOptimizer {
public:
    DpStSpeedOptimizer()=delete;
    DpStSpeedOptimizer(const PathData& pathdata):
            discretizedPath_(pathdata.discretized_path()),frenetFramePath_(pathdata.frenet_frame_path()){}
    ~DpStSpeedOptimizer()= default;

    bool Process(const PathData& path_data,
                 const common::TrajectoryPoint& init_point,
//                 const common::ReferenceLine& reference_line,
                 vector< PathObstacle> &path_obstacles,
                 SpeedData* const speed_data) ;
    bool CombinePathAndSpeedProfile(
            const double relative_time, const double start_s,
            const PathData &path_data_,SpeedData *speed_data,
            common::DiscretizedTrajectory* ptr_discretized_trajectory);
private:
    bool SearchStGraph(const vector< PathObstacle>& path_obstacles,
                       const SpeedLimitDecider& speed_limit_decider,
                       const PathData& path_data, SpeedData* speed_data
                       //PathDecision* path_decision,
                       //planning_internal::STGraphDebug* debug
    ) const;
    bool CreateObstacle(vector< PathObstacle> *path_obstacles);

private:
    common::TrajectoryPoint init_point_;
//    const common::ReferenceLine* reference_line_ = nullptr;
    SLBoundary adc_sl_boundary_;
    DpStSpeedConfig dp_st_speed_config_;
    StBoundaryConfig st_boundary_config_;
    common::DiscretizedPath discretizedPath_;
    common::FrenetFramePath frenetFramePath_;
};


#endif //LATTICEPLAN_SP_ST_OPTIMIZER_H
