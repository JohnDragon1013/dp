//
// Created by jydragon on 18-8-7.
//

#ifndef LATTICEPLAN_ST_PATH_OBSTACLE_TEST_H
#define LATTICEPLAN_ST_PATH_OBSTACLE_TEST_H


#include "../dp_plan/Obstacle.h"
#include "../dp_plan/common.h"
#include "st_boundary.h"

class PathObstacle : public Obstacle {
    //这里打算手动来创建障碍物，包括动态和静态的，来做一下st-graph或者lattice
public:
    PathObstacle() = delete;
    PathObstacle(common::DiscretizedPath discretizedPath,common::FrenetFramePath frenetFramePath,
                 float v , float length,float width, common::PathPoint location):
            discretizedPath_(discretizedPath),
            frenetFramePath_(frenetFramePath),
            length_(length),width_(width),
            speed_(v),location_(location){}
    ~PathObstacle() = default;

    bool CreateObstacle();

    SLBoundary GetSLBoundary(double dt);
    StBoundary GetStBoundary();

    vector<common::PathPoint> location4drae(double dt);
    float v()const{ return speed_;};
    float angle()const{ return location_.theta;};
    float length()const{ return length_;};

    const common::PathPoint locataion() const{ return location_;};

//    void SetStBoundary(StBoundary st_boundary){st_boundary_ = st_boundary;}
//    void SetSlBoundary(SLBoundary sl_boundary){perception_sl_boundary_ = sl_boundary;}

private:
    void calculateCorner(vector<common::PathPoint> *boundaryCorner,const common::PathPoint loca);
    void calculateFinalPosition();
    void calculateStBoundary();
    common::SLPoint translatexy2sl(common::PathPoint sl);
    void updateLocation(common::PathPoint& lastlocation,double dt);
    double findStartTime(common::PathPoint obs_location);
    bool findcollisionTime(double &s_station,double &e_station,double &s_time,double &e_time);
    //直接以平行于参考路径的方式来创建障碍物。
//    SLBoundary perception_sl_boundary_;

    StBoundary reference_line_st_boundary_;
//    StBoundary st_boundary_;
    common::DiscretizedPath discretizedPath_;
    common::FrenetFramePath frenetFramePath_;
    float speed_; //动态障碍物的信息
    float length_;
    float width_;
    const common::PathPoint location_;
    DpStSpeedConfig stSpeedConfig_;
};


#endif //LATTICEPLAN_ST_PATH_OBSTACLE_TEST_H
