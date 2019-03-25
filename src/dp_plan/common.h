//
// Created by jydragon on 18-7-20.
//

#ifndef LATTICEPLAN_COMMON_H
#define LATTICEPLAN_COMMON_H

#include <string.h>
//#include <asdl.h>
#include <vector>
#include "../BasicStruct.h"

namespace common {

    struct SLPoint {
        double s=0.0;
        double l=0.0;
    };
    struct FrenetFramePoint {
        double s=0.0;
        double l=0.0;
        double dl=0.0;
        double ddl=0.0;
    };
    struct SpeedPoint {
        double s;
        double t;
        // speed (m/s)
        double v;
        // acceleration (m/s^2)
        double a;
        // jerk (m/s^3)
        double da;
    };

    struct PathPoint {
        // coordinates
        double x;
        double y;
        double z;

        // direction on the x-y plane
        double theta;
        // curvature on the x-y planning
        double kappa;
        // accumulated distance from beginning of the path
        double s;

        double l;

        // derivative of kappa w.r.t s.
        double dkappa;
        // derivative of derivative of kappa w.r.t s.
        // double ddkappa = 8;
        // The lane ID where the path point is on
        //string lane_id;
    };
    struct TrajectoryPoint {
        TrajectoryPoint(){};
        TrajectoryPoint(double v_,double a_):v(v_),a(a_),relative_time(0){}
        // path point
        PathPoint path_point;

        // linear velocity
        double v=0.0;  // in [m/s]
        // linear acceleration
        double a=0.0;
        // relative time from beginning of the trajectory
        double relative_time;
    };
    struct DpPolyPathConfig {
        //double sample_points_num_each_level=9;
        //double step_length_max ;
        //double step_length_min ;
        float lateral_sample_offset;
        float lateral_sample_offset_step;
        //double lateral_adjust_coeff;
        double eval_time_interval ;
        float path_resolution ;
        float obstacle_ignore_distance ;
        //double obstacle_collision_distance;
        //double obstacle_risk_distance ;
        float obstacle_collision_cost ;
        float historical_l_cost ;//建议在100到200之间
        float path_l_cost ;
        float path_dl_cost;
        float path_ddl_cost;
        float path_l_cost_param_l0 ;
        float path_l_cost_param_b;
        float path_l_cost_param_k;
        float path_out_lane_cost ;
        float path_end_l_cost ;
    };
    struct ReferencePoint{
        double x=0.0;
        double y=0.0;
        double s=0.0;
        double l=0.0;
        double heading =0.0;
        double kappa_ = 0.0;
        double dkappa_ = 0.0;
    };
    struct  ReferenceLine{
        ReferenceLine() = default;
        ReferenceLine(const ReferenceLine& reference_line) = default;
        struct SpeedLimit {
            double start_s = 0.0;
            double end_s = 0.0;
            double speed_limit = 0.0;  // unit m/s
            SpeedLimit() = default;
            SpeedLimit(double _start_s, double _end_s, double _speed_limit)
                    : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
        };
        std::vector<SpeedLimit> speed_limit_;
        std::vector<ReferencePoint> reference_points_;
        double Length;//参考路径长度

    };
    ;
    typedef std::vector<common::FrenetFramePoint> FrenetFramePath;
    typedef std::vector<PathPoint> DiscretizedPath;

    //还需要一个参考路径的信息
    static bool SLToXY(ReferenceLine refer_lane,FrenetFramePath frenetFramePath,DiscretizedPath &discretizedPath) {

    }

    /**
     * not use now !!!!
     * @param refer_lane
     * @param discretized Path
     * @param frenetFramePath
     * @return
     */
    static bool XYToSL(ReferenceLine refer_lane,DiscretizedPath discretizedPath,FrenetFramePath &frenetFramePath) {
        if(refer_lane.reference_points_.empty()||discretizedPath.empty())
            return false;
        double minD=MAXD;
        for(auto dp:discretizedPath)
        {
            int index =-1;
            for(int i=0;i<refer_lane.reference_points_.size();i++)
            {
                double dis = BasicStruct::Distance(dp,refer_lane.reference_points_[i]);
                if(dis<minD)
                {
                    minD=dis;
                    index =i;
                }
            }
            RoadPoint org(refer_lane.reference_points_[index].x,refer_lane.reference_points_[index].y,
                          BasicStruct::AngleNormalnize1(PI/2-refer_lane.reference_points_[index].heading),0);
            double xout,yout;
            BasicStruct::WorldtoMap(org,dp.x,dp.y,xout,yout);
            FrenetFramePoint Fpoint;
            Fpoint.s=refer_lane.reference_points_[index].s+yout;//有误差，不过应该也不太大
            Fpoint.l=xout;//正负需要测试一下。
            frenetFramePath.push_back(Fpoint);
        }
        return true;
    }



}
#endif //LATTICEPLAN_COMMON_H
