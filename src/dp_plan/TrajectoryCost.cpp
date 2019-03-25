//
// Created by jydragon on 18-7-21.
//

#include <functional>
#include "TrajectoryCost.h"
TrajectoryCost::TrajectoryCost(
        const common::DpPolyPathConfig &config,
        const common::ReferenceLine reference_line,
        const Obstacle obs,
        const common::SLPoint &init_sl_point,
        const common::FrenetFramePath lastSLpath)
        : config_(config),
          reference_line_(reference_line),
          init_sl_point_(init_sl_point),
          m_obs(obs),
        m_lastpath(lastSLpath)
{

    const float total_time =3;//随便填的
            //std::min(heuristic_speed_data_.TotalTime(), FLAGS_prediction_total_time);

    num_of_time_stamps_ = static_cast<uint32_t>(
            std::floor(total_time / config.eval_time_interval));

//    for (const auto *ptr_path_obstacle : obstacles) {
//        if (ptr_path_obstacle->IsIgnore()) {
//            continue;
//        } else if (ptr_path_obstacle->LongitudinalDecision().has_stop()) {
//            continue;
//        }
//        const auto &sl_boundary = ptr_path_obstacle->PerceptionSLBoundary();
//
//        //车的左右宽度
//
//        const auto *ptr_obstacle = ptr_path_obstacle->obstacle();
//    }
}

ComparableCost TrajectoryCost::CalculatePathCost(
        const QuinticPolynomialCurve1d &curve, const float start_s,
        const float end_s, const uint32_t curr_level, const uint32_t total_level) {
    ComparableCost cost;
    float path_cost = 0.0;
    std::function<float(const float)> quasi_softmax = [this](const float x) {
        const float l0 = this->config_.path_l_cost_param_l0;//1.5
        const float b = this->config_.path_l_cost_param_b;//0.4
        const float k = this->config_.path_l_cost_param_k;//1.5
        return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
    };

    //获取车辆大小信息 用car代替即可
//    const auto &vehicle_config =
//            common::VehicleConfigHelper::instance()->GetConfig();
        Car vehiclemodel;
    const float width = 2.1;//vehicle_config.vehicle_param().width();

    for (float curve_s = 0.0; curve_s < (end_s - start_s);
         curve_s += config_.path_resolution) {
        const float l = curve.Evaluate(0, curve_s);

        path_cost += l * l * config_.path_l_cost * quasi_softmax(std::fabs(l));

        const float dl = std::fabs(curve.Evaluate(1, curve_s));
        path_cost += dl * dl * config_.path_dl_cost;

        const float ddl = std::fabs(curve.Evaluate(2, curve_s));
        path_cost += ddl * ddl * config_.path_ddl_cost;
    }
    path_cost *= config_.path_resolution;//分辨率  步长

    if (curr_level == total_level-1) {//TODO :never step into,find the reason
        const float end_l = curve.Evaluate(0, end_s - start_s);
        path_cost += end_l*end_l * config_.path_end_l_cost;
    }
    cost.smoothness_cost = path_cost;
    return cost;
}

ComparableCost TrajectoryCost::CalculateHistoricalCost(
        const QuinticPolynomialCurve1d &curve, const float start_s, const float end_s,
        const uint32_t curr_level, const uint32_t total_level)
{
    ComparableCost cost;
    if(curr_level>=m_lastpath.size()||m_lastpath.empty()) {
        return cost;
    }

    const float l = curve.Evaluate(0, end_s-start_s);
    common::FrenetFramePoint sl;
    for(const auto &i:m_lastpath)
    {
        if(i.s>end_s)
            break;
        sl =i;
    }
    double dis_ =fabs(l-sl.l);
    cost.historical_cost = dis_ * config_.historical_l_cost;
    return cost;
}


double Sigmoid(const double value) { return 1 / (1 + std::exp(-1.0 * value)); }
ComparableCost TrajectoryCost::CalculateStaticObstacleCost(
        const QuinticPolynomialCurve1d &curve, const float start_s,
        const float end_s)
{
    ComparableCost obstacle_cost;

    int maxCarsize;
    bool stopiter=false;
    //计算障碍物cost： 是否碰撞 先用一个包围盒判断，成功直接返回默认空值。
    //离障碍物距离 ：S、L
    int index_last;
    double lasts=0.0;
    int count=1;
    for (float curr_s = start_s; curr_s <= end_s;
         curr_s += config_.path_resolution)
    {//遍历曲线
        const float curr_l = curve.Evaluate(0, curr_s - start_s);
        const float d_prime = curve.Evaluate(1, curr_s - start_s);
        const float d_pprime = curve.Evaluate(1, curr_s - start_s);
        //找到最近的参考点
        int indexXY=0;
        for(auto eachSLpoint:reference_line_.reference_points_)
        {
            if(eachSLpoint.s>curr_s)
                break;
            indexXY++;
        }
        auto referpoint = reference_line_.reference_points_[indexXY];
        //index_last=index;
        lasts = curr_s;
        //获得s l转换为笛卡尔
//速度和加速度随便输的
        std::array<double, 3> s_conditions = {curr_s, 5, 0.5};
        std::array<double, 3> d_conditions = {curr_l, d_prime, d_pprime};
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double kappa = 0.0;
        double v = 0.0;
        double a = 0.0;
         CartesianFrenetConverter::frenet_to_cartesian(
                curr_s,referpoint.x,referpoint.y,referpoint.heading,referpoint.kappa_,referpoint.dkappa_,
                s_conditions,d_conditions,&x, &y,
                &theta, &kappa, &v, &a);
        RoadPoint ptemp(x,y,theta,kappa);

        //for (int cariter=0;cariter<10;++cariter)
        {
            Carsize[0].Position=ptemp;
            RoadPoint co;
            if(m_obs.DetectGridObs(Carsize[0],co))
            {
                //maxCarsize =cariter;
                //if(cariter ==0)
                {
                    obstacle_cost.cost_items[ComparableCost::HAS_COLLISION] = true;
                    stopiter =true;
                    break;
                }
//                break;
            }

            count++;
        }
        if (stopiter)
            break;
    }

//    const float delta_s = (maxCarsize+1)*0.2 +2.1/2.0;
//    obstacle_cost.safety_cost +=
//            config_.obstacle_collision_cost *
//            Sigmoid(config_.obstacle_collision_distance - delta_s);//距离越大，cost越小
//    obstacle_cost.safety_cost *= 20*config_.path_resolution;
    return obstacle_cost;
}
//
//ComparableCost TrajectoryCost::GetCostFromObsSL(const float adc_s, const float adc_l) {
////    const auto &vehicle_param =
////            common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
//
//    ComparableCost obstacle_cost;
//
//    //车头到中心
//    const float adc_front_s = adc_s + vehicle_param.front_edge_to_center();
//    const float adc_end_s = adc_s - vehicle_param.back_edge_to_center();
//    const float adc_left_l = adc_l + vehicle_param.left_edge_to_center();
//    const float adc_right_l = adc_l - vehicle_param.right_edge_to_center();
//
//    const double FLAGS_static_decision_nudge_l_buffer =0.2;//格网误差
//    bool no_overlap = ((adc_front_s < obs_sl_boundary.start_s() ||
//                        adc_end_s > obs_sl_boundary.end_s()) ||  // longitudinal
//                       (adc_left_l  + FLAGS_static_decision_nudge_l_buffer <
//                        obs_sl_boundary.start_l() ||
//                        adc_right_l - FLAGS_static_decision_nudge_l_buffer >
//                        obs_sl_boundary.end_l()));  // lateral
//
//    //有碰撞
//    if (!no_overlap) {
//        obstacle_cost.cost_items[ComparableCost::HAS_COLLISION] = true;
//    }
//
//    // if obstacle is behind ADC, ignore its cost contribution. 返回空值
//    if (adc_front_s > obs_sl_boundary.end_s()) {
//        return obstacle_cost;
//    }
//
//    //车辆中心点到障碍物中心的L距离
//    const float delta_l = std::fabs(
//            adc_l - (obs_sl_boundary.start_l() + obs_sl_boundary.end_l()) / 2.0);
//
//    obstacle_cost.safety_cost +=
//            config_.obstacle_collision_cost *
//            Sigmoid(config_.obstacle_collision_distance - delta_l);//0 or 1
//
//    //车辆中心点到障碍物中心的S距离
//    const float delta_s = std::fabs(
//            adc_s - (obs_sl_boundary.start_s() + obs_sl_boundary.end_s()) / 2.0);
//    obstacle_cost.safety_cost +=
//            config_.obstacle_collision_cost *
//            Sigmoid(config_.obstacle_collision_distance - delta_s);
//    return obstacle_cost;
//}


void TrajectoryCost::setCarsize(){
    for(int i=0;i<10;i++)
    {
        Carsize[i].length=4.2+i*0.2;
        Carsize[i].width =2.3+i*0.2;
        Carsize[i].RtoT =0.8;
    }
}


// TODO(All): optimize obstacle cost calculation time
ComparableCost TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve,
                                         const float start_s, const float end_s,
                                         const uint32_t curr_level,
                                         const uint32_t total_level)
{
    setCarsize();
    ComparableCost total_cost;
    // path cost
    total_cost += CalculatePathCost(curve, start_s, end_s, curr_level, total_level);
    //cout<<"PathCost:"<<total_cost.smoothness_cost<<" ";
    // static obstacle cost
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);
    total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);
    gettimeofday(&t2,NULL);
    //level可能需要调整。
    total_cost += CalculateHistoricalCost(curve, start_s, end_s, curr_level, total_level);
    auto deltaT = (t2.tv_sec - t1.tv_sec) * 1000000 + t2.tv_usec - t1.tv_usec;//微秒
    //cout<<"Obstacle check time:"<<deltaT<<"us"<<endl;
    //cout<<"ObstacleCost:"<<total_cost.safety_cost<<endl;
    // dynamic obstacle cost
    //total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
    return total_cost;
}


