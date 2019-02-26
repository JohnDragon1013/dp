//
// Created by jydragon on 18-8-1.
//

#ifndef LATTICEPLAN_ST_GRAPH_H
#define LATTICEPLAN_ST_GRAPH_H

//#include "../dp_plan/common.h"
#include "st_graph_point.h"
#include "st_graph_data.h"
#include "st_cost.h"
#include "../dp_plan/Obstacle.h"
#include "st_boundary.h"
class DpStGraph {
public:
    DpStGraph(const StGraphData& st_graph_data, const DpStSpeedConfig& dp_config,
              //const std::vector<const PathObstacle*>& obstacles,
              const common::TrajectoryPoint& init_point
             );

    bool Search(SpeedData* const speed_data);

private:
    bool InitCostTable();
//检索速度文件
    bool RetrieveSpeedProfile(SpeedData* const speed_data);

    bool CalculateTotalCost();
    void CalculateCostAt(const uint32_t c, const uint32_t r);

    float CalculateEdgeCost(const STPoint& first, const STPoint& second,
                            const STPoint& third, const STPoint& forth,
                            const float speed_limit);
    float CalculateEdgeCostForSecondCol(const uint32_t row,
                                        const float speed_limit);
    float CalculateEdgeCostForThirdCol(const uint32_t curr_r,
                                       const uint32_t pre_r,
                                       const float speed_limit);

    void GetRowRange(const StGraphPoint& point, int* highest_row,
                     int* lowest_row);

private:
    const StGraphData& st_graph_data_;

    // dp st configuration
    DpStSpeedConfig dp_st_speed_config_;

    // obstacles based on the current reference line
    //const std::vector<const PathObstacle*>& obstacles_;
    Obstacle m_obstacle;

    // vehicle configuration parameter
   // const common::VehicleParam& vehicle_param_ =
     //       common::VehicleConfigHelper::GetConfig().vehicle_param();

    // initial status
    common::TrajectoryPoint init_point_;

    // cost utility with configuration;
    DpStCost dp_st_cost_;

//    const SLBoundary& adc_sl_boundary_;

    float unit_s_ = 0.0;
    float unit_t_ = 0.0;

    // cost_table_[t][s]
    // row: s, col: t --- NOTICE: Please do NOT change.
    std::vector<std::vector<StGraphPoint>> cost_table_;


};


#endif //LATTICEPLAN_ST_GRAPH_H
