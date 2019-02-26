//
// Created by jydragon on 18-8-7.
//

#include "st_path_obstacle_test.h"
#include "../CartesianFrenetConverter.h"

SLBoundary PathObstacle::GetSLBoundary(double dt) {
    SLBoundary perception_sl_boundary_;
    perception_sl_boundary_.start_s = std::numeric_limits<double >::max();
    perception_sl_boundary_.end_s =-99999999;//std::numeric_limits<double >::min();
    perception_sl_boundary_.start_l =std::numeric_limits<double >::max();
    perception_sl_boundary_.end_l =-99999999;//std::numeric_limits<double >::min();
    if(speed_>0 && dt>=0)
    {
        vector<common::PathPoint> corners_;
        vector<common::SLPoint> slpoints;
        common::PathPoint newlocation;
        newlocation.x = location_.x + speed_* dt *cos(location_.theta);
        newlocation.y = location_.y + speed_* dt *sin(location_.theta);
        newlocation.theta = location_.theta;
        calculateCorner(&corners_,newlocation);
        for(const auto &pa:corners_)
        {
            slpoints.emplace_back(translatexy2sl(pa));
        }

        for (auto &corner : slpoints) {
            perception_sl_boundary_.end_s = std::fmax(corner.s, perception_sl_boundary_.end_s);
            perception_sl_boundary_.end_l = std::fmax(corner.l, perception_sl_boundary_.end_l);
            perception_sl_boundary_.start_s = std::fmin(corner.s, perception_sl_boundary_.start_s);
            perception_sl_boundary_.start_l = std::fmin(corner.l, perception_sl_boundary_.start_l);
        }
    }
    else
    {
        cout<<"This obstacle's velocity is "<<speed_<<endl;
    }
    return perception_sl_boundary_;
}

StBoundary PathObstacle::GetStBoundary()  {
    //将该障碍物在total time内变成一个长矩形，判断和路径是否有相交，有则记录相交的时间
//    common::PathPoint final_Location;
//    final_Location.x =location_.x + /*stSpeedConfig_.total_time*/20* speed_ *cos(location_.theta);
//    final_Location.y =location_.y + /*stSpeedConfig_.total_time*/20* speed_ *sin(location_.theta);
//    final_Location.theta = location_.theta;
//    vector<common::PathPoint> corners_1,corners_2;
//    calculateCorner(&corners_1,final_Location);
//    calculateCorner(&corners_2,location_);
//    DynamicObstacle polygonObstacle;
//    polygonObstacle.Polygon[0] = RoadPoint(corners_1[0].x,corners_1[0].y,0,0);
//    polygonObstacle.Polygon[1] = RoadPoint(corners_1[1].x,corners_1[1].y,0,0);
//    polygonObstacle.Polygon[2] = RoadPoint(corners_2[2].x,corners_2[2].y,0,0);
//    polygonObstacle.Polygon[3] = RoadPoint(corners_2[3].x,corners_2[3].y,0,0);
//    m_Obs  = (polygonObstacle);
//    Car Polygon_detect;

    double s_station,e_station;
    double s_time,e_time;
    findcollisionTime(s_station,e_station,s_time,e_time);
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    point_pairs.emplace_back(std::make_pair(STPoint(s_station,s_time),
                                            STPoint(e_station,e_time)));
    return StBoundary(point_pairs);
//    calculateStBoundary();
//    if(reference_line_st_boundary_.IsEmpty())
//        return StBoundary();
//    else
//        return reference_line_st_boundary_;

}
double PathObstacle::findStartTime(common::PathPoint pathPoint_){
    //将时间慢慢累加
    RoadPoint org = g_currentLocation;
    org.angle = BasicStruct::AngleNormalnize1(PI/2.0-g_currentLocation.angle);
    double  xout,yout;
    BasicStruct::WorldtoMap(org,pathPoint_.x,pathPoint_.y,xout,yout);
    RoadPoint pp(xout,yout,pathPoint_.theta,pathPoint_.kappa);

    double delta_t = 0.0;
    common::PathPoint obs_location = location_;

    Car mycar;
    mycar.Position = pp;//RoadPoint(location_.x,location_.y,location_.theta,0.0);
    mycar.rearx=mycar.Position.x;
    mycar.reary=mycar.Position.y;
    mycar.phi=mycar.Position.angle;

    RoadPoint LeftRear,LeftFront,RightRear,RightFront;

    LeftRear.x=mycar.rearx-mycar.RtoT*cos(mycar.phi)-0.5*mycar.width*sin(mycar.phi);
    LeftRear.y=mycar.reary-mycar.RtoT*sin(mycar.phi)+0.5*mycar.width*cos(mycar.phi);
    LeftFront.x=mycar.rearx+(mycar.length-mycar.RtoT)*cos(mycar.phi)+0.5*mycar.width*sin(mycar.phi);
    LeftFront.y=mycar.reary+(mycar.length-mycar.RtoT)*sin(mycar.phi)-0.5*mycar.width*cos(mycar.phi);
    RightRear.x=mycar.rearx-mycar.RtoT*cos(mycar.phi)+0.5*mycar.width*sin(mycar.phi);
    RightRear.y=mycar.reary-mycar.RtoT*sin(mycar.phi)-0.5*mycar.width*cos(mycar.phi)	;
    RightFront.x=mycar.rearx+(mycar.length-mycar.RtoT)*cos(mycar.phi)-0.5*mycar.width*sin(mycar.phi);
    RightFront.y=mycar.reary+(mycar.length-mycar.RtoT)*sin(mycar.phi)+0.5*mycar.width*cos(mycar.phi);

    vector<double> vertX1,vertY1;
    vertX1.push_back(LeftRear.x);
    vertX1.push_back(LeftFront.x);
    vertX1.push_back(RightFront.x);
    vertX1.push_back(RightRear.x);

    vertY1.push_back(LeftRear.y);
    vertY1.push_back(LeftFront.y);
    vertY1.push_back(RightFront.y);
    vertY1.push_back(RightRear.y);

    bool flag_=false;
    while(!flag_){
        vector<double> vertX2,vertY2;
        vector<common::PathPoint> cor_;
        calculateCorner(&cor_,obs_location);
        for(auto co:cor_)
        {
            vertX2.emplace_back(co.x);
            vertY2.emplace_back(co.y);
        }
        flag_ = BasicStruct::PolygonOverPolygon(vertX1,vertY1,vertX2,vertY2);
        delta_t += 0.1;
        if(delta_t>=20)
            break;

        updateLocation(obs_location,delta_t);
    }

    if(!flag_)
        return 20.0;
    return delta_t;
}
bool PathObstacle::findcollisionTime(double &s_station,double &e_station,double &s_time,double &e_time){
    //将时间累加
    double delta_t = 0.0;

    bool flag_findstart=false;
    s_time =20.0;e_time =20.0;
    s_station =frenetFramePath_.back().s;
    e_station =frenetFramePath_.back().s;
    double kEpsilon = 1e-2;
    while(true){
        delta_t += 0.1;
        SLBoundary obs_track = GetSLBoundary(delta_t);
        if(obs_track.start_s>= frenetFramePath_.front().s&&obs_track.end_s<=frenetFramePath_.back().s)//不超出路径范围
        {
            double minl = std::min(std::fabs(obs_track.start_l),std::fabs(obs_track.end_l));
            if(minl<=kEpsilon)
                continue;
            if(minl<1.1|| (minl>=1.1 && obs_track.end_l>0 && obs_track.start_l<0 ) ) {
                if(!flag_findstart) {
                    s_time = delta_t;
                    s_station = obs_track.start_s;
                    flag_findstart = true;
                }
            }
            else{
                if(flag_findstart)
                {
                    e_time = delta_t;
                    e_station = obs_track.end_s;
                    break;
                }
            }
        } else continue;

        if(delta_t>=20.0)
            break;
    }
    if(delta_t>=20)
        return false;
    return true;
}
void PathObstacle::calculateCorner(vector<common::PathPoint> *corners_,const common::PathPoint loca) {
    const double dx1 = cos(loca.theta) * length_/2.0;
    const double dy1 = sin(loca.theta) * length_/2.0;
    const double dx2 = sin(loca.theta) * width_/2.0;
    const double dy2 = -cos(loca.theta) * width_/2.0;
    corners_->clear();
    corners_->emplace_back(loca.x + dx1 - dx2, loca.y + dy1 - dy2,0.0,0.0);//左前
    corners_->emplace_back(loca.x + dx1 + dx2, loca.y + dy1 + dy2,0.0,0.0);//右前
    corners_->emplace_back(loca.x - dx1 + dx2, loca.y - dy1 + dy2,0.0,0.0);//右后
    corners_->emplace_back(loca.x - dx1 - dx2, loca.y - dy1 - dy2,0.0,0.0);//左后
}

common::SLPoint PathObstacle::translatexy2sl(common::PathPoint xy) {
    //默认为 两种坐标的路径长度是一致的
    int index =0;
    double mindis =std::numeric_limits<double >::max();
    for(int i=0;i<discretizedPath_.size();++i) {
        double dis = BasicStruct::Distance_2(discretizedPath_[i], xy);
        if (dis < mindis) {
            index = i;
            mindis = dis;
        }
    }

    if(index>=frenetFramePath_.size())
    {
        cout<<"the discretized path and frenet path are different length.."<<endl;
    }
    auto referpoint = frenetFramePath_[index];
    double s,l;
    CartesianFrenetConverter::cartesian_to_frenet(
            referpoint.s,discretizedPath_[index].x,discretizedPath_[index].y,discretizedPath_[index].theta,
            xy.x,xy.y,
            &s,&l);
    return common::SLPoint(s,l);
}

void PathObstacle::updateLocation(common::PathPoint &lastlocation,double dt) {
    lastlocation.x+= speed_* dt *cos(lastlocation.theta);
    lastlocation.y+= speed_* dt *sin(lastlocation.theta);
}

bool PathObstacle::CreateObstacle() {
    return false;
}

vector<common::PathPoint> PathObstacle::location4drae(double dt) {
    dt=dt>20?20:dt;
    if(speed_>0 && dt>=0) {
        vector<common::PathPoint> corners_;
        vector<common::SLPoint> slpoints;
        common::PathPoint newlocation;
        newlocation.x = location_.x + speed_ * dt * cos(location_.theta);
        newlocation.y = location_.y + speed_ * dt * sin(location_.theta);
        newlocation.theta = location_.theta;
        calculateCorner(&corners_, newlocation);
//        for (const auto &pa:corners_) {
//            slpoints.emplace_back(translatexy2sl(pa));
//        }
        return corners_;//
    } else
        return vector<common::PathPoint>();
}

void PathObstacle::calculateStBoundary() {
    double s_station,e_station;
    double s_time,e_time;
    findcollisionTime(s_station,e_station,s_time,e_time);
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    point_pairs.emplace_back(std::make_pair(STPoint(s_station,s_time),
                                            STPoint(e_station,e_time)));
    reference_line_st_boundary_ = StBoundary(point_pairs);
}

//void PathObstacle::calculateFinalPosition(vector<common::PathPoint> *corners_) {
//
//}
