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
        SLPoint() = default;
        SLPoint(double ss,double ll):s(ss),l(ll){}
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
        PathPoint() = default;
        PathPoint(double x_,double y_,double theta_,double kappa_):x(x_),y(y_),theta(theta_),kappa(kappa_),s(0),dkappa(0){}
        // coordinates
        double x;
        double y;

        // direction on the x-y plane
        double theta=0;
        // curvature on the x-y planning
        double kappa;
        // accumulated distance from beginning of the path
        double s;

        // derivative of kappa w.r.t s.
        double dkappa;
        // derivative of derivative of kappa w.r.t s.
         double ddkappa = 0;
        // The lane ID where the path point is on
        //string lane_id;
    };
    struct TrajectoryPoint {
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
        int sample_points_num_each_level=9;
        double step_length_max =15.0;
        double step_length_min =8.0;
        double lateral_sample_offset=0.5;
        double lateral_adjust_coeff=0.5 ;
        double eval_time_interval =0.1 ;
        double path_resolution =0.1;
        double obstacle_ignore_distance =20.0;
        double obstacle_collision_distance=0.2;
        double obstacle_risk_distance =2.0;
        double obstacle_collision_cost=1e3 ;
        double historical_l_cost = 200;//建议在100到200之间
        double path_l_cost =6.5;
        double path_dl_cost =8e3;
        double path_ddl_cost=5e1;
        double path_l_cost_param_l0 =1.5;
        double path_l_cost_param_b=0.40;
        double path_l_cost_param_k=1.5;
        double path_out_lane_cost =1e8;
        double path_end_l_cost =1.0e4;
        double sidepass_distance =2.8;
        int navigator_sample_num_each_level =3;
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
    typedef std::vector<common::FrenetFramePoint> FrenetFramePath;
    typedef std::vector<PathPoint> DiscretizedPath;
    typedef std::vector<TrajectoryPoint> DiscretizedTrajectory;

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
