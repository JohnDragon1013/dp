//
// Created by jydragon on 18-8-4.
//

#include <iostream>
#include "sp_st_optimizer.h"
#include "st_graph.h"


bool DpStSpeedOptimizer::Process(  const PathData& path_data, //路径数据 离散点、sl点
                                   const common::TrajectoryPoint& init_point,//路径起点 相当于当前位置
//                                   const common::ReferenceLine& reference_line, //参考路径
                                   //PathDecision* const path_decision,//路径决策信息 之后会存储 障碍物
                                   vector< PathObstacle> &path_obstacles,
                                   SpeedData* const speed_data) {

    init_point_ = init_point;

    if (path_data.discretized_path().empty()) {
        std::string msg("Empty path data");
        cout << msg;
        return false;
    }

    if(path_obstacles.empty()){
        CreateObstacle(&path_obstacles)  ;
        const std::string msg =
                "Creating obstacle for dp st speed optimizer failed.";
        cout << msg;
        return false;//Status(ErrorCode::PLANNING_ERROR, msg);
    }

    SpeedLimitDecider speed_limit_decider(st_boundary_config_, path_data);

    if (!SearchStGraph(path_obstacles, speed_limit_decider, path_data, speed_data)) {
        const std::string msg("Failed to search graph with dynamic programming.");
        cout << msg;
        return false;
    }
    return true;
}

bool DpStSpeedOptimizer::SearchStGraph(const vector< PathObstacle>& path_obstacles,
        const SpeedLimitDecider& speed_limit_decider,
        const PathData &path_data, SpeedData *speed_data) const {
    std::vector<const StBoundary*> boundaries;
    vector< StBoundary> obs_stboundaries(path_obstacles.size());
    int i =0 ;
    for( auto p_obs:path_obstacles)
    {
        obs_stboundaries[i] =p_obs.GetStBoundary(); //之后改成const 这里有点多余
        const StBoundary* st= &obs_stboundaries[i];
        boundaries.push_back(st);
        i++;
    }


    // step 2 perform graph search
    SpeedLimit speed_limit;
    if (!speed_limit_decider.GetSpeedLimits(path_obstacles, &speed_limit)) {
        cout << "Getting speed limits for dp st speed optimizer failed!";
        return false;
    }

    const float path_length = path_data.frenet_frame_path().back().s;
    StGraphData st_graph_data(boundaries, init_point_, speed_limit, path_length);

    DpStGraph st_graph(st_graph_data, dp_st_speed_config_,
            //reference_line_info_->path_decision()->path_obstacles().Items(),
            init_point_);
//计算最优cost
    if (!st_graph.Search(speed_data)) {
        cout << "failed to search graph with dynamic programming.";
        return false;
    }
    return true;
}
bool DpStSpeedOptimizer::CombinePathAndSpeedProfile(
        const double relative_time, const double start_s,
        const PathData &path_data_,SpeedData *speed_data,
        common::DiscretizedTrajectory* ptr_discretized_trajectory) {
//    CHECK(ptr_discretized_trajectory != nullptr);
    // use varied resolution to reduce data load but also provide enough data
    // point for control module
//    const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
//    const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
//    const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;
    if (path_data_.discretized_path().empty()) {
        cout << "path data is empty";
        return false;
    }
    //按时间序列，依次匹配pathpoint和速度。
//    auto discretized_path = path_data_.discretized_path();
    for (double cur_rel_time = 0.0; cur_rel_time < speed_data->TotalTime();
         cur_rel_time += 0.1)//(cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion : kSparseTimeResolution))
    {
        SpeedPoint speed_point;
        if (!speed_data->EvaluateByTime(cur_rel_time, &speed_point)) {
            cout << "Fail to get speed point with relative time " << cur_rel_time;
            return false;
        }

        if (speed_point.s > path_data_.frenet_frame_path().back().s) {
            break;
        }
        common::PathPoint path_point;
        if (!path_data_.GetPathPointWithPathS(speed_point.s, &path_point)) {
            cout << "Fail to get path data with s " << speed_point.s
                   << "path total length " << path_data_.frenet_frame_path().back().s;
            return false;
        }
        path_point.s = path_point.s + start_s;

        common::TrajectoryPoint trajectory_point;
        trajectory_point.path_point =path_point;
        trajectory_point.v = speed_point.v;
        trajectory_point.a = speed_point.a;
        trajectory_point.relative_time = speed_point.t + relative_time;//现在的时间
        ptr_discretized_trajectory->push_back(trajectory_point);
    }
    return true;
}
bool DpStSpeedOptimizer::CreateObstacle(vector< PathObstacle> *path_obstacles) {
    //创建两个动态障碍物，一个静态障碍物
    //两种坐标的路径，

    common::PathPoint loca(15,1,2.0,0.0);
    //float v , float length , float width , common::PathPoint location
    path_obstacles->emplace_back( PathObstacle(discretizedPath_,frenetFramePath_,3.0,5.0,2.4,loca));
    common::PathPoint loca2(-10,5,PI/3.0,0.0);
    path_obstacles->emplace_back( PathObstacle(discretizedPath_,frenetFramePath_,3.5,5.0,2.4,loca2));

//    common::PathPoint loca3(5,20,PI/2.0,0.0);
//    path_obstacles->emplace_back( PathObstacle(discretizedPath_,frenetFramePath_,0.0,5.0,2.4,loca3));

    return true;
}
