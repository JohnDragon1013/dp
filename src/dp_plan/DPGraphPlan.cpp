//
// Created by jydragon on 18-7-20.
//

#include <algorithm>
#include <iostream>
#include <fstream>
#include "DPGraphPlan.h"

bool DPGraphPlan::SamplePathWaypoints(const common::TrajectoryPoint &init_point) {
    if(reference_line_.reference_points_.empty())
        return false;
    //apollo最小采样的距离为40米
    //采样间隔最小为8米，最大15米，根据初速度来确定
    const double kMinSampleDistance = 40.0;
    // 总长度 = min(初始点 + max(初始速度 × 8, 最小采样距离), 参考线长度）
//    const float total_length = std::fmin(
//            init_sl_point_.s + std::fmax(init_point.v * 8.0, kMinSampleDistance)
    const float total_length= reference_line_.Length;

    constexpr float kSamplePointLookForwardTime = 4.0;
    // 采样步长 = 初始速度 × 采样前视时长，要求：
    // step_length_min(默认值：8) <= 采样步长 <= step_length_max(默认值：15)
    float step_length =10.0;//采样步长其实应该根据车速来选择 效果较好。
           //std::fmin( std::fmax(init_point.v * kSamplePointLookForwardTime, 8), 15);

    // 累计轨迹弧长
    float accumulated_s = init_sl_point_.s;
    float prev_s = accumulated_s;
    vector<vector<common::SLPoint>> as;
    while(accumulated_s<total_length)//纵向
    {
        vector<common::SLPoint> levelPoints;
        accumulated_s +=step_length;
        if(accumulated_s>(total_length+step_length/2.0))//超出总长度过多
            break;
        const float kDefaultUnitL =1.0;//半米一个点 默认横向偏移
        for (float i = -4; i <= 4; ) {
            common::SLPoint samplepoint;
            samplepoint.s=accumulated_s;
            samplepoint.l =i;
            levelPoints.push_back(samplepoint);
            i+=kDefaultUnitL;
        }
        as.push_back(levelPoints);
        if(accumulated_s>=50)
            step_length =15.0;
    }
    m_AllSamplePoints=as;
    if(m_AllSamplePoints.empty())
        return false;
    else
        return true;
}

void DPGraphPlan::UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                             const int level, const int total_level,
                             TrajectoryCost *trajectory_cost,
                             DPRoadGraphNode *front,
                             DPRoadGraphNode *cur_node) {
    // 生成当前点到前一level所有航点的的曲线 5次曲线
//    ofstream logfile("/home/jydragon/catkin_ws/src/latticePlan/src/PathCost.txt",ios::app);
    double sum_historical=1.0;
    double sum_smoothness=1.0;
    double sum_obstacle=1.0;
//    vector<ComparableCost> whole_level_cost;
//    vector<QuinticPolynomialCurve1d> whole_level_curve;
//    logfile<<"level:"<<level<<endl;
    for(auto &prev_dp_point:prev_nodes) {
        const auto &prev_sl_point = prev_dp_point.sl_point;
        const auto &cur_point = cur_node->sl_point;
        float init_dl = 0.0;
        float init_ddl = 0.0;
        if (level == 1) {
            init_dl = init_frenet_frame_point_.dl;
            init_ddl = init_frenet_frame_point_.ddl;
        }
        QuinticPolynomialCurve1d curve(prev_sl_point.l, init_dl, init_ddl,
                                       cur_point.l, 0.0, 0.0,
                                       cur_point.s - prev_sl_point.s);
        if(!IsValidCurve(curve))//判断曲线是否生成
            return;
        //计算到起点的cost
        const auto cost =
                trajectory_cost->Calculate(curve, prev_sl_point.s, cur_point.s,
                                           level, total_level) + prev_dp_point.min_cost;
//        whole_level_cost.emplace_back(cost);
//        whole_level_curve.emplace_back(curve);
//        sum_smoothness += cost.smoothness_cost;
//        sum_historical += cost.historical_cost;
//        sum_obstacle += cost.safety_cost;
        // 根据代价最小的原则，在前一level所有航点中找到与当前点连接代价最小的航点，
        // 存储结果
        cur_node->UpdateCost(&prev_dp_point, curve, cost);
        if(cur_node->min_cost.cost_items[ComparableCost::HAS_COLLISION])
            cur_node->min_cost_prev_node = nullptr;
//        logfile<<cost.smoothness_cost<<" "<<cost.historical_cost<< " " ;
    }
//    logfile<<"\nsum_smoothness:"<<sum_smoothness<<"   sum_obstacle:"<<sum_obstacle<<endl;
//    logfile.close();
    //是否有更优父节点
    // 尝试将当前点直接连接到初始点，看其代价是否比当前点到前一level航点的最小代价还小，
    // 若小于则将最小代价航点更新。这种情况一般只会存在于改变车道的情形。
    // try to connect the current point with the first point directly
    if (level >= 2) {
        const float init_dl = init_frenet_frame_point_.dl;
        const float init_ddl = init_frenet_frame_point_.ddl;
        QuinticPolynomialCurve1d curve(init_sl_point_.l, init_dl, init_ddl,
                                       cur_node->sl_point.l, 0.0, 0.0,
                                       cur_node->sl_point.s - init_sl_point_.s);
        if (!IsValidCurve(curve)) {//判断曲线是否生成
            return;
        }
        const auto cost = trajectory_cost->Calculate(
                curve, init_sl_point_.s, cur_node->sl_point.s, level, total_level);
        cur_node->UpdateCost(front, curve, cost);
        if(cur_node->min_cost.cost_items[ComparableCost::HAS_COLLISION])
            cur_node->min_cost_prev_node = nullptr;
    }
}

bool DPGraphPlan::GenerateMinCostPath(std::vector<DPRoadGraphNode> *min_cost_path) {

    if (!SamplePathWaypoints(init_point_ ) || m_AllSamplePoints.size() < 1) {
        std::cout << "Fail to sample path waypoints! reference_line_length = "
               << reference_line_.Length<<endl;
        return false;
    }
    // 轨迹代价
    TrajectoryCost trajectory_cost(config_, reference_line_,m_obstacles,  init_sl_point_ ,m_lastFrenetPath);
    //将起点加到路点的最前面
    m_AllSamplePoints.insert(m_AllSamplePoints.begin(),
                          std::vector<common::SLPoint>{init_sl_point_});
    // 最小代价值路图节表点链表
    std::list<std::list<DPRoadGraphNode>> graph_nodes;
    graph_nodes.emplace_back();
    //插入起点
    graph_nodes.back().emplace_back(init_sl_point_, nullptr, ComparableCost());
    auto &front = graph_nodes.front().front();
    size_t total_level = m_AllSamplePoints.size();
    for (size_t level = 1; level < total_level; ++level) {
        const auto &prev_dp_nodes = graph_nodes.back();
        const auto &level_points = m_AllSamplePoints[level];

        graph_nodes.emplace_back();
        //从后一个点向前一个点生成曲线
        for (int num = 0; num < level_points.size(); ++num) {
            const auto &cur_point = level_points[num];

            graph_nodes.back().emplace_back(cur_point, nullptr);
            auto &cur_node = graph_nodes.back().back();
            //生成到之前cost最小的路径 最后存在graph_nodes.back().back();
            UpdateNode(prev_dp_nodes, level, total_level, &trajectory_cost, &front,
                       &cur_node);
        }
    }
    cout<<"                 gengxinwanccc"<<endl;
    // graph_nodes.back()（即最后一条航点链表）
    // find best path
    DPRoadGraphNode fake_head;
    //遍历最后一个level 找出cost最小的那个，等于fake_head
    for (const auto &cur_dp_node : graph_nodes.back()) {
        fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
                             cur_dp_node.min_cost);
    }
    const auto *fatherNode =&fake_head;
    std::vector<RoadPoint> repath;
    while(fatherNode->min_cost_prev_node)
    {
        fatherNode =fatherNode->min_cost_prev_node;
        min_cost_path->push_back(*fatherNode);
        repath.emplace_back(fatherNode->sl_point.l,fatherNode->sl_point.s,0.0,0.0);
    }
    //找到起点

    if (fatherNode != &graph_nodes.front().front()) {
        cout<<" 最优路径没有导向起点。。。。。。。"<<endl;
        return false;
    }
    //cout<<"   xxxxxxx"<<endl;
    std::reverse(min_cost_path->begin(),min_cost_path->end());
    std::reverse(repath.begin(),repath.end());
    g_LastPath.pps.swap(repath);

    vector<vector<DPRoadGraphNode>> apn;
    int xx=0;
    int yy =0;
    for(const auto &cur_dp_node : graph_nodes.back()){
        xx++;
        vector<DPRoadGraphNode> tempnode;
        const auto *dp_node_ = &cur_dp_node;
        while (dp_node_->min_cost_prev_node){
            tempnode.push_back(*dp_node_);
            dp_node_ =dp_node_->min_cost_prev_node;
        }
        if(dp_node_==&graph_nodes.front().front())
            yy++;
        tempnode.push_back(*dp_node_);
        std::reverse(tempnode.begin(),tempnode.end());
        apn.push_back(tempnode);
    }
    //cout<<" 最后一行的点数量："<<xx<<" 到达起点的数量："<<yy<<endl;
    m_AllpathNode = apn;
    return true;
}

std::vector<common::FrenetFramePoint> DPGraphPlan::FindPathTunnel(std::vector<DPRoadGraphNode> min_cost_path) {
    std::vector<common::FrenetFramePoint> frenet_path;//最终的路径
    float accumulated_s = init_sl_point_.s;//累计的s
    const float path_resolution =10* config_.path_resolution;

    for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
        const auto &prev_node = min_cost_path[i - 1];
        const auto &cur_node = min_cost_path[i];

        const float path_length = cur_node.sl_point.s - prev_node.sl_point.s;
        float current_s = 0.0;
        const auto &curve = cur_node.min_cost_curve;
        //这里具体的终止条件，可以改变最后待选路径，两个节点间的间距。。。
        while (current_s + path_resolution /*2.0*/ < path_length) {
            ////l dl ddl 三阶
            const float l = curve.Evaluate(0, current_s);
            const float dl = curve.Evaluate(1, current_s);
            const float ddl = curve.Evaluate(2, current_s);
            common::FrenetFramePoint frenet_frame_point;

            frenet_frame_point.s= (accumulated_s + current_s);
            frenet_frame_point.l=(l);
            frenet_frame_point.dl=(dl);
            frenet_frame_point.ddl=(ddl);
            frenet_path.push_back(std::move(frenet_frame_point));
            current_s += path_resolution;
        }
        //长度可能不一致
//        if (i == min_cost_path.size() - 1) {
//            accumulated_s += current_s;
//        } else
            {
            accumulated_s += path_length;
        }
    }
    if(frenet_path.size()<2)
        cout<<" **********提取的路径是空的"<<endl;
    return  frenet_path;
    //FrenetFramePath tunnel(frenet_path);
    //path_data->SetReferenceLine(&reference_line_);
    //path_data->SetFrenetPath(tunnel);

}
bool DPGraphPlan::IsValidCurve(const QuinticPolynomialCurve1d &curve) const {
    constexpr float kMaxLateralDistance = 20.0;
    for (float s = 0.0; s < curve.ParamLength(); s += 2.0) {
        const float l = curve.Evaluate(0, s);
        if (std::fabs(l) > kMaxLateralDistance) {
            return false;
        }
    }
    return true;
}
bool DPGraphPlan::ChooseMinCostPath(std::list<std::list<DPRoadGraphNode>> graph_nodes,std::vector<DPRoadGraphNode> *min_cost_path) {
    // graph_nodes.back()（即最后一条航点链表）就是我们所需的最小代价航点链表
    // find best path
    std:: vector< vector<DPRoadGraphNode> > apn;
    vector<double > dis2lastpath;
    int in = g_LastPath.pps.size()-1;
    double sum_cost=0.0;
    if(in>0)
    {
        for(const auto &cur_dp_node : graph_nodes.back())
        {
            const auto *fatherNode =&cur_dp_node;
            double sum_dis = 0.0;
            vector<DPRoadGraphNode> tempnode;
            while(fatherNode->min_cost_prev_node)
            {
                sum_dis += fatherNode->sl_point.l -g_LastPath.pps[in].x;//横向距离
                fatherNode =fatherNode->min_cost_prev_node;
                tempnode.push_back(*fatherNode);
                in--;
                if(in<0)
                    break;
            }
            tempnode.push_back(*fatherNode);
            std::reverse(tempnode.begin(),tempnode.end());
            apn.push_back(tempnode);
            dis2lastpath.push_back(sum_dis);
            sum_cost += cur_dp_node.min_cost.safety_cost;
        }
    }
    m_AllpathNode = apn;

    DPRoadGraphNode fake_head;int ii=0;
    //遍历最后一个level 找出cost最小的那个，等于fake_head的前一个节点
    for (const auto &cur_dp_node : graph_nodes.back()) {
        //这里可能需要手写
        //cur_dp_node.min_cost.historical_cost = dis2lastpath[ii];
        fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
                             cur_dp_node.min_cost);

        if(1)
        {

        }
        ii++;
    }

    const auto *fatherNode =&fake_head;
    while(fatherNode->min_cost_prev_node)
    {
        fatherNode =fatherNode->min_cost_prev_node;
        min_cost_path->push_back(*fatherNode);
    }
    //找到起点

    if (fatherNode != &graph_nodes.front().front()) {
        return false;
    }
    std::reverse(min_cost_path->begin(),min_cost_path->end());
    //这里就找完终点了 需要加入上一次的路径，进行对比。

    return true;
}


PathData DPGraphPlan::Getfinalpath(common::FrenetFramePath &lastFrenetPath) {
    PathData path_data_;
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);
    std::vector<DPRoadGraphNode> min_cost_path;
//    cout<<"开始生成路径"<<endl;
    if(!GenerateMinCostPath(&min_cost_path))
        return PathData();
    else
        cout<<"生成最小cost路径成功"<<endl;
    common::FrenetFramePath finalFrenetpath=FindPathTunnel(min_cost_path);

    if(finalFrenetpath.empty())
        return PathData();
    lastFrenetPath = finalFrenetpath;
    path_data_.SetFrenetPath(finalFrenetpath);//to the path data(save frenet path)
    common::DiscretizedPath path;
    int index_last=0;
    double lasts=0.0;

    bool outrange=0;
    for (auto frenetp:finalFrenetpath)
    {
        int finalindex =0;
        for(auto referSLPoint:reference_line_.reference_points_)
        {
            if(referSLPoint.s>frenetp.s) {
                break;
            }

            finalindex++;
        }
        if(finalindex>reference_line_.reference_points_.size()-1)
        {
            finalindex = reference_line_.reference_points_.size()-1;
            outrange = true;
            //cout<<"#####################"<<endl;
            break;
        }
        auto referpoint = reference_line_.reference_points_[finalindex];
        //获得s l转换为笛卡尔
//速度和加速度随便输的
        std::array<double, 3> s_conditions = {frenetp.s, 5, 0.5};
        std::array<double, 3> d_conditions = {frenetp.l, frenetp.dl, frenetp.ddl};
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double kappa = 0.0;
        double v = 0.0;
        double a = 0.0;
        CartesianFrenetConverter::frenet_to_cartesian(
                frenetp.s,referpoint.x,referpoint.y,referpoint.heading,0.0,0.0,
                s_conditions,d_conditions,&x, &y,
                &theta, &kappa, &v, &a);

        common::PathPoint ptemp(x,y,theta,kappa);
        ptemp.s = frenetp.s;
        if(x==0.0&&y==0.0)
            cout<<"aa"<<endl;
        path.push_back(ptemp);
    }

    //这里用来提取所有路径。
    vector<vector<common::FrenetFramePoint>> AllpathFrenetPoint;
    for (int i = 0; i < m_AllpathNode.size(); ++i) {
        std::vector<common::FrenetFramePoint> temp_;
        temp_ =FindPathTunnel(m_AllpathNode[i]);
        AllpathFrenetPoint.push_back(temp_);
    }
    vector<PathPointXY> apxy;
    for (auto perFrenetpath:AllpathFrenetPoint) {
        PathPointXY temp_;
        outrange =false;
        for (auto frenetp:perFrenetpath) {
            int mayindex=0;
            for(auto referSLPoint:reference_line_.reference_points_)
            {
                if(referSLPoint.s>frenetp.s) {
                    break;
                }
                mayindex++;

            }
            if(mayindex>=reference_line_.reference_points_.size()-1)
            {
                mayindex = reference_line_.reference_points_.size()-1;
                outrange = true;
                //cout<<"|||||||||||||||||||||"<<endl;
                break;
            }
            //找到最近的参考点
            auto referpoint =reference_line_.reference_points_[mayindex];// referXYPath.pps[mayindex];
            //获得s l转换为笛卡尔
//速度和加速度随便输的
            std::array<double, 3> s_conditions = {frenetp.s, 5, 0.5};
            std::array<double, 3> d_conditions = {frenetp.l, frenetp.dl, frenetp.ddl};
            double x = 0.0;
            double y = 0.0;
            double theta = 0.0;
            double kappa = 0.0;
            double v = 0.0;
            double a = 0.0;
            CartesianFrenetConverter::frenet_to_cartesian(
                    frenetp.s, referpoint.x, referpoint.y, referpoint.heading, 0.0, 0.0,
                    s_conditions, d_conditions, &x, &y,
                    &theta, &kappa, &v, &a);
            RoadPoint ptemp(x, y, theta, kappa);
            temp_.pps.push_back(ptemp);
        }
        apxy.push_back(temp_);
    }
    m_AllxyPath = apxy;
    //cout<<"最后提取的路径数量："<<m_AllxyPath.size()<<endl;
//用来提取其他路径
    if(path.empty())
    {
        return PathData();
    }
    path_data_.SetDiscretizedPath(path);
    gettimeofday(&t2, NULL);
    auto deltaT = (t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec;//微秒
    cout<< "deltat:" << deltaT / 1000<<endl;
    return path_data_;
}

