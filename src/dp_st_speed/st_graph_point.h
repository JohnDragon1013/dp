//
// Created by jydragon on 18-8-1.
//

#ifndef LATTICEPLAN_ST_GRAPH_POINT_H
#define LATTICEPLAN_ST_GRAPH_POINT_H

#include "SpeedData.h"

class StGraphPoint {
public:
    int index_s() const;
    int index_t() const;

    const STPoint& point() const;
    const StGraphPoint* pre_point() const;

    float reference_cost() const;
    float obstacle_cost() const;
    float total_cost() const;

    void Init(const unsigned int index_t, const unsigned int index_s,
              const STPoint& st_point);

    // given reference speed profile, reach the cost, including position
    void SetReferenceCost(const float reference_cost);

    // given obstacle info, get the cost;
    void SetObstacleCost(const float obs_cost);

    // total cost
    void SetTotalCost(const float total_cost);

    void SetPrePoint(const StGraphPoint& pre_point);

private:
    STPoint point_;
    const StGraphPoint* pre_point_ = nullptr; //previous stpoint
    unsigned int index_s_ = 0;
    unsigned int index_t_ = 0;

    float reference_cost_ = 0.0;
    float obstacle_cost_ = 0.0;
    float total_cost_ = 100000000.0;//std::numeric_limits<float>::infinity();
};


#endif //LATTICEPLAN_ST_GRAPH_POINT_H
