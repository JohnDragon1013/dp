//
// Created by jydragon on 18-7-20.
//

#ifndef LATTICEPLAN_DPGRAPHPLAN_H
#define LATTICEPLAN_DPGRAPHPLAN_H

#include <limits>
#include <list>
#include "../QuinticPolynomialCurve1d.h"

#include "TrajectoryCost.h"
#include "common.h"
#include <fstream>


class DPGraphPlan {
    struct DPRoadGraphNode {
    public:
        DPRoadGraphNode() = default;

        DPRoadGraphNode(const common::SLPoint point_sl,
                        const DPRoadGraphNode *node_prev)
                : sl_point(point_sl), min_cost_prev_node(node_prev) {}

        DPRoadGraphNode(const common::SLPoint point_sl,
                        const DPRoadGraphNode *node_prev,
                        const ComparableCost &cost)
                : sl_point(point_sl), min_cost_prev_node(node_prev), min_cost(cost) {}

        void UpdateCost(const DPRoadGraphNode *node_prev,
                        const QuinticPolynomialCurve1d &curve,
                        const ComparableCost &cost) {
            if (cost <= min_cost) {
                min_cost = cost;
                min_cost_prev_node = node_prev;
                min_cost_curve = curve;
            }
        }

        common::SLPoint sl_point;
        const DPRoadGraphNode *min_cost_prev_node = nullptr;
        ComparableCost min_cost = {true, true, true,
                                   std::numeric_limits<float>::infinity(),
                                   std::numeric_limits<float>::infinity(),
                                   std::numeric_limits<float>::infinity()};
        QuinticPolynomialCurve1d min_cost_curve;
    };
public:
    DPGraphPlan()= default;
    DPGraphPlan(const common::DpPolyPathConfig &config,
                const PathPointxy referLanexy,
                const common::TrajectoryPoint startpoint,
                const common::ReferenceLine &reference_line_info,
                const Obstacle ob,
                common::FrenetFramePath lastSLpath
    ):reference_line_(reference_line_info),
      referXYPath(referLanexy),
      init_point_(startpoint),
      m_obstacles(ob),
      config_(config),
      m_lastFrenetPath(lastSLpath)
    {
        log.open("PlanData.txt");
    };
    ~DPGraphPlan()= default;
    PathPointxy Getfinalpath(common::FrenetFramePath &lastFrenetPath);
    //inline std::vector<common::FrenetFramePoint> Getpath(){ return m_fanalFrenetpath;};
    inline vector<PathPointxy> GetAllPath(){return m_AllxyPath;};
private:
    void UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                    const int level, const int total_level,
                    TrajectoryCost *trajectory_cost,
                    DPRoadGraphNode *front,
                    DPRoadGraphNode *cur_node);
    bool SamplePathWaypoints(const common::TrajectoryPoint &init_point);
    bool GenerateMinCostPath(std::vector<DPRoadGraphNode> *min_cost_path);
    bool ChooseMinCostPath(std::list<std::list<DPRoadGraphNode>> graph_nodes,std::vector<DPRoadGraphNode> *min_cost_path);
    double StepbyObstacle();
    std::vector<common::FrenetFramePoint> FindPathTunnel(std::vector<DPRoadGraphNode> _path);//最后生成路径之后进行评价
    void logpath(const PathPointxy &pa);
    common::DpPolyPathConfig config_;
    common::TrajectoryPoint init_point_;//路径点
    const PathPointxy referXYPath;//xy坐标
    const common::ReferenceLine reference_line_;//SL坐标
    //SpeedData speed_data_;
    common::SLPoint init_sl_point_;//起点
    common::FrenetFramePoint init_frenet_frame_point_;//起点
    vector<vector<common::SLPoint>> m_AllSamplePoints;

    common::FrenetFramePath m_lastFrenetPath;//最终的路径

    Obstacle m_obstacles;

    bool IsValidCurve(const QuinticPolynomialCurve1d &curve) const;
    vector<vector<DPRoadGraphNode>> m_AllpathNode;
    vector<PathPointxy> m_AllxyPath;
    ofstream log;
};


#endif //LATTICEPLAN_DPGRAPHPLAN_H
