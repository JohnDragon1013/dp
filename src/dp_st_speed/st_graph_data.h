//
// Created by jydragon on 18-8-1.
//

#include "../dp_plan/common.h"
#include "speed_limit.h"
#ifndef LATTICEPLAN_ST_GRAPH_DATA_H
#define LATTICEPLAN_ST_GRAPH_DATA_H

#endif //LATTICEPLAN_ST_GRAPH_DATA_H
class StGraphData {
public:
    StGraphData(//const std::vector<const StBoundary*>& st_boundaries,
                const common::TrajectoryPoint& init_point,
                const SpeedLimit& speed_limit, const double path_data_length)
            : //st_boundaries_(st_boundaries),
              init_point_(init_point),
              speed_limit_(speed_limit),
              path_data_length_(path_data_length) {};
    StGraphData() = default;

   // const std::vector<const StBoundary*>& st_boundaries() const{return st_boundaries_;}

    const common::TrajectoryPoint& init_point() const{ return init_point_;}

    const SpeedLimit& speed_limit() const{ return speed_limit_;}

    double path_data_length() const{ return path_data_length_;}

private:
    //std::vector<const StBoundary*> st_boundaries_;
    common::TrajectoryPoint init_point_;

    SpeedLimit speed_limit_;
    double path_data_length_ = 0.0;
};