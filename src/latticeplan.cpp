#include "ros/ros.h"
#include "std_msgs/String.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/filereadstream.h"
#include <rapidjson/istreamwrapper.h>
#include "dp_plan/Obstacle.h"
#include "dp_plan/LocationReceiver.h"
#include "dp_plan/DPGraphPlan.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <pthread.h>
#include "msgHeader/Follow.h"
PathPointxy ExcutablePath;
vector<PathPointxy> wholeReferLane;
PathPointxy referLane;
Obstacle obs;
LocationReceiver Location_receiver;

RoadPoint initialPoint;
RoadPoint lastLocation;
RoadPoint collisionPoint;
vector<PathPointxy> path4draw;
common::FrenetFramePath last_path_;
void send4Draw();
void send4follow(bool flag_failed);
bool beginPlan();
void readconfig(common::DpPolyPathConfig &config);
ros::Publisher pcl_pubPath ;
ros::Publisher pcl_pubOtherPath ;
ros::Publisher pcl_pubReferPath ;
ros::Publisher pubFollow;

bool flag_PlanSucceed=false;
double TravelingDis = 0;
ros::Time _time_update;
ros::Time _time_register;
void send4Draw(){
    //发送路径信息
    //路径信息
    visualization_msgs::Marker line_path, obsCloud, points_refer,lane_path;
    visualization_msgs::Marker crashPoint;
    lane_path.header.frame_id =crashPoint.header.frame_id = points_refer.header.frame_id = obsCloud.header.frame_id = line_path.header.frame_id = "velodyne";
    crashPoint.header.stamp = points_refer.header.stamp = obsCloud.header.stamp = line_path.header.stamp = ros::Time::now();
    crashPoint.ns =points_refer.ns = obsCloud.ns = line_path.ns = "Plan_Data";
    line_path.id = 0;
    line_path.type = visualization_msgs::Marker::LINE_STRIP;
    line_path.scale.x = 0.1;
    line_path.color.r = 1.0;
    line_path.color.b = 1.0;
    line_path.color.a = 1.0;

    //障碍物信息
    //pcl::PointCloud<pcl::PointXYZ> cloud_obs;
    //sensor_msgs::PointCloud2 obs_output;
    obsCloud.id = 1;
    obsCloud.type = visualization_msgs::Marker::POINTS;
    obsCloud.scale.x = 0.2;
    obsCloud.scale.y = 0.2;
    obsCloud.color.g = 1.0;
    obsCloud.color.a = 1.0;
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //sensor_msgs::PointCloud2 output;

    //参考路径信息
    points_refer.id = 2;
    points_refer.type = visualization_msgs::Marker::POINTS;
    points_refer.scale.x = 0.25;
    points_refer.scale.y = 0.25;
    points_refer.color.b = 1.0;
    points_refer.color.a = 1.0;
    //pcl::PointCloud<pcl::PointXYZRGB> cloud_refer;
    //sensor_msgs::PointCloud2 refer_output;
    //碰撞点信息
    crashPoint.id = 3;
    crashPoint.type = visualization_msgs::Marker::POINTS;
    crashPoint.scale.x = 0.5;
    crashPoint.scale.y = 0.5;
    crashPoint.color.r = 1.0;
    crashPoint.color.a = 1.0;
    //pcl::PointCloud<pcl::PointXYZRGB> cloud_colli;
    //sensor_msgs::PointCloud2 co_output;
    //车道线
    lane_path.id=4;
    lane_path.type = visualization_msgs::Marker::POINTS;
    lane_path.scale.x = 0.3;
    lane_path.scale.y = 0.1;
    lane_path.color.r = 0;
    lane_path.color.g = 0;
    lane_path.color.b = 0;
    lane_path.color.a = 1.0;
    //其他路径
    pcl::PointCloud<pcl::PointXYZRGB> other_Path;
    sensor_msgs::PointCloud2 otherPath_output;

    //参考路径
    pcl::PointCloud<pcl::PointXYZRGB> reference_Path;
    sensor_msgs::PointCloud2 reference_output;

    pcl::PointXYZRGB collsionPo;
    geometry_msgs::Point co;
    co.x = collisionPoint.y;
    co.y = -collisionPoint.x;
    co.z = 0;
    //collsionPo.r = 250;
    crashPoint.points.push_back(co);
    //ROS_INFO("begin to send path");
    //将路径坐标转换为局部
    for (size_t i = 0; i < ExcutablePath.pps.size(); ++i) {
        double xout, yout;
        RoadPoint org = g_currentLocation;
        org.angle = BasicStruct::AngleNormalnize1(PI / 2.0 - g_currentLocation.angle);
        BasicStruct::WorldtoMap(org, ExcutablePath.pps[i].x, ExcutablePath.pps[i].y, xout, yout);
        geometry_msgs::Point p;
        p.x = yout;
        p.y = -xout;
        p.z = 0;
        line_path.points.push_back(p);
    }
    //将参考路径转换为局部坐标 TODO:添加车道线
    for (size_t i = 0; i < referLane.pps.size(); ++i) {
        double xout, yout;
        RoadPoint org = g_currentLocation;
        org.angle = BasicStruct::AngleNormalnize1(PI / 2.0 - g_currentLocation.angle);
        BasicStruct::WorldtoMap(org, referLane.pps[i].x, referLane.pps[i].y, xout, yout);
        //geometry_msgs::Point temp;
        pcl::PointXYZRGB temp;
        temp.x = yout;
        temp.y = -xout;
        temp.z = 0;
        temp.b =200.0;
        reference_Path.points.push_back(temp);
        geometry_msgs::Point p;
        p.x = temp.x;
        p.y = temp.y-1.5;
        p.z = temp.z;
        lane_path.points.push_back(p);
        p.y = temp.y+1.5;
        lane_path.points.push_back(p);
    }

    vector<unsigned char> griddata = obs.GetGridObs();
    if (griddata.size() > 0) {
        //cloud_obs.width = 600;//ExcutablePath.pps.size();
        //cloud_obs.height = 1000;
        //cloud_obs.points.resize(cloud_obs.width * cloud_obs.height);

        for (int j = 0; j < 400; ++j) {
            for (int k = 0; k < 150; ++k) {
                geometry_msgs::Point temp;
                if (griddata[j * 150 + k] >= 1) {
                    temp.x = j * GRID_WIDTH - 40;//平移100米
                    temp.y = -(k * GRID_WIDTH - 15);//平移60米
                    temp.z = 0;
                    obsCloud.points.push_back(temp);

                } else {}
            }
        }

    } else {
    }
    //cout<<"发送显示的cloud_obs 大小："<<obsCloud.points.size()<<" griddata大小："<<endl;
    for (int l = 0; l < path4draw.size(); ++l) {
        geometry_msgs::Point lastp;
        //if(l==0||l==8)
        {
            for (int m = 0; m < path4draw[l].pps.size(); ++m) {
                double xout, yout;
                RoadPoint org = g_currentLocation;
                org.angle = BasicStruct::AngleNormalnize1(PI / 2.0 - g_currentLocation.angle);
                BasicStruct::WorldtoMap(org, path4draw[l].pps[m].x, path4draw[l].pps[m].y, xout, yout);
                pcl::PointXYZRGB p;
                p.x = yout;
                p.y = -xout;
                p.z = 0;
                p.a=1.0;
                p.r = 1;
                p.b = 2;
                other_Path.points.push_back(p);
            }
        }
    }
    pcl::toROSMsg(other_Path, otherPath_output);
    otherPath_output.header.frame_id = "velodyne";
    pcl::toROSMsg(reference_Path, reference_output);
    reference_output.header.frame_id = "velodyne";
    pcl_pubPath.publish(obsCloud);

    pcl_pubPath.publish(crashPoint);
//    pcl_pubReferPath.publish(points_refer);
    pcl_pubPath.publish(line_path);
//    pcl_pubPath.publish(lane_path);

    pcl_pubOtherPath.publish(otherPath_output);
    pcl_pubReferPath.publish(reference_output);
    //cout << "发送完成。。。" << endl;

}
//ofstream iput("/home/jydragon/catkin_ws/src/test/src/path.txt");


bool map_invalid(vector<PathPointxy> lanes,int curin,int targetin){
    if(lanes.empty()|| curin<0 || targetin <0 || curin >= lanes.size() || targetin >= lanes.size())
        return true;
    else
        return false;
}
int curIndex;
int targetIndex;
double velocity;
common::DpPolyPathConfig config;
bool beginPlan(){
    referLane = wholeReferLane[targetIndex];
    //RoadPoint s(0,-5,0,0);
        //要将xy转换为sl
    common::ReferenceLine reference_line_info;
    if(CartesianFrenetConverter::XY2SL(referLane,reference_line_info)) {
        auto init = CartesianFrenetConverter::initial_Point_trans(reference_line_info.reference_points_.front(),g_currentLocation,velocity,0,0);
        init.path_point.l=0;//仿真实验，真实环境需要注释掉
        DPGraphPlan dpp(config, referLane, init, reference_line_info, obs ,last_path_);
        //cout << "start Plan" << endl;
        PathPointxy ep = dpp.Getfinalpath(last_path_);
        path4draw = dpp.GetAllPath();
        //bool flag_plan = ;
        if(ep.pps.empty()) {
            ROS_WARN("PLAN failed");
        }
        else
            ROS_INFO("PLAN success");
        ExcutablePath =ep;
    }
    else{
        PathPointxy ep;
        ExcutablePath =ep;//置为空
        vector<PathPointxy> p4;
        path4draw = p4;
        ROS_WARN("can't translate reference lane from XY to SL");
    }
    return ExcutablePath.pps.empty()? false:true;
}
bool Replan(){
    bool flag_pathempty = 0;		// 规划路径为空触发重规划标志
    bool flag_pathcollision = 0;	// 规划路径有障碍物触发重规划标志
    bool flag_traveldis = 0;		// 行驶距离触发的重规划标志（主动触发策略）
    bool flag_cartopath = 0;		// 车偏离规划路径触发的重规划标志
    int collisionpoint;
    double collisionDis;
    flag_pathempty = ExcutablePath.pps.empty();
    flag_pathcollision = Location_receiver.ObstacleonLane(ExcutablePath, collisionpoint, collisionDis);
    // 掉头模式下碰撞检测只需要考虑一定距离内的，因为需要掉头时，当前方向上有障碍物，无法通行，这里采用了 3m
    // 有障碍情况下，是否需要重规划的策略判断
    int m_Threholdtraveldis = ExcutablePath.length * 0.25;
    flag_traveldis = (TravelingDis > m_Threholdtraveldis);
    flag_cartopath = Location_receiver.DisCurrentToPath(ExcutablePath) > 2.0;
    if (!flag_PlanSucceed || flag_pathempty || flag_pathcollision || flag_traveldis ||
        flag_cartopath )//|| flag_changeLane )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
void Process(){
    while(ros::ok())
    {
        usleep(1e3);
        g_currentLocation = Location_receiver.GetCurrentLocation();// np;
        TravelingDis = BasicStruct::Distance(lastLocation,g_currentLocation);
        try {
            wholeReferLane = Location_receiver.GetMap(curIndex,targetIndex,velocity,obs);//Location_receiver.GetLocalPath();
            //cout<<"map: "<<targetIndex<<"\t";
        }
        catch(...)
        {
            ROS_WARN("referLane update failed... size::[%d]",wholeReferLane.size());
            continue ;
        }
        if(map_invalid(wholeReferLane,curIndex,targetIndex))
        {
            ROS_WARN("map invalid!");
            continue ;
        }
        if(!Location_receiver.reSolveRosMsg())
        {
            //cout<<"no lidar"<<endl;
            //continue;
        }
        //add the replan function
        if(Replan())
        {
            flag_PlanSucceed = beginPlan();
            lastLocation = g_currentLocation;
        //plan finished ,storage the data.
        //Location_receiver.logfile();
        }
        //send4follow(flag_failed);

        send4Draw();
    }
}
int main(int argc, char **argv)
{
    //common::DpPolyPathConfig config;
    readconfig(config);
    //Location_receiver.readPath();//读取一次路径信息
    ros::init(argc, argv, "test_ljy");//0711 测试RRT注掉
    ros::NodeHandle n;//一个node貌似可以接收多个消息
    // 发布
    pcl_pubPath = n.advertise<visualization_msgs::Marker> ("Path_output", 1);
    pcl_pubOtherPath =n.advertise<sensor_msgs::PointCloud2>("OtherPath",1);
    pcl_pubReferPath = n.advertise<sensor_msgs::PointCloud2> ("Reference_Lane",1);
    //发布局部规划路径给pathfollow
    pubFollow = n.advertise<iau_ros_msgs::Follow>("IAU/Follow", 20);
//订阅
    //订阅激光雷达的障碍物网格数据
    ros::Subscriber sub_lidar = n.subscribe("IAU/Grid",10,&LocationReceiver::LidarCallback,&Location_receiver);
    //订阅定位数据，gps+imu之后由position发出
    ros::Subscriber sub_Location = n.subscribe("IAU/Location",10,&LocationReceiver::LocationCallback,&Location_receiver);

    //订阅地图模块发送的数据
    ros::Subscriber subMap = n.subscribe("IAU/Map", 10,&LocationReceiver::MapCallback,&Location_receiver);

    //订阅车辆状态信息
    ros::Subscriber sub_vehicleStatus = n.subscribe("IAU/VehicleStatus",10,&LocationReceiver::VehicleStatusCallback,&Location_receiver);

    //订阅地图模块在匹配不到地图时的全局地图数据
    //ros::Subscriber subGlobalMap = nh.subscribe("IAU/GlobalMap", 10, GlobalMap_message);

    //订阅地图模块发布的逆向车道信息
    //ros::Subscriber subReverseLane = n.subscribe("IAU/Reverse", 10, reverseMap_message);
    //订阅地图模块发布的建议速度信息
    ros::Subscriber subVelocity = n.subscribe("IAU/MapVelocity", 10, &LocationReceiver::VelocityCallback,&Location_receiver);
    //


    _time_update= ros::Time();
    _time_register =ros::Time();//-.now().toSec()
    //boost::function0<void> f = boost::bind(&send4Draw,1);
    boost::thread thread_(&Process);
    thread_.detach();
    std::cout<<"Start Listening!"<<std::endl;
    //MsgCallback(n);
    while (ros::ok())
    {
        //Location_receiver.logfile();
        usleep(1e3);
        ros::spinOnce();
    }
    //sleep(1);
    return 0;
}
void send4follow(bool flag_failed){
    iau_ros_msgs::Follow PFollow;
    if (flag_failed) {
//        if (DistoObs < 5) {
//            PFollow.force = 1;
        //} else {
            PFollow.force = 1 << 1;
            PFollow.speed = 0;
        //}
    } else {
        PFollow.force = 1 << 1;
        PFollow.speed;/// = g_SendSpeed;///
    }
    PFollow.cur_pos.x = g_currentLocation.x;
    PFollow.cur_pos.y = g_currentLocation.y;
    PFollow.cur_pos.yaw = g_currentLocation.angle;

    PFollow.cur_pos.x += initialPoint.x;
    PFollow.cur_pos.y += initialPoint.y;
    for (auto& pt : ExcutablePath.pps) {
        iau_ros_msgs::PointXYA pp;
        pp.x = pt.x;
        pp.y = pt.y;
        pp.yaw = pt.angle;
        PFollow.points.push_back(pp);
    }
    PFollow.num = PFollow.points.size();
    pubFollow.publish(PFollow);
}

void readconfig(common::DpPolyPathConfig &config){
    ifstream ifs("config.json");
    rapidjson::IStreamWrapper isw(ifs);
    rapidjson::Document doc;
    doc.ParseStream(isw);

    if (doc.HasParseError()) {
        rapidjson::ParseErrorCode code = doc.GetParseError();
        cout<<"JSON ParseErrorCode"<<(code)<<endl;
        return;
    }
    //rapidjson::Value &node1 = doc["sample_points_num_each_level"];
    //config.sample_points_num_each_level = doc.FindMember("sample_points_num_each_level")->value.GetInt();
    //config.step_length_max = doc["step_length_max"].GetFloat();
    //config.step_length_min = doc["step_length_min"].GetFloat();
    config.lateral_sample_offset = doc["lateral_sample_offset"].GetFloat();
    config.lateral_sample_offset_step = doc["lateral_sample_offset_step"].GetFloat();
    config.eval_time_interval =doc["eval_time_interval"].GetFloat();
    config.path_resolution = doc["path_resolution"].GetFloat();
    config.obstacle_ignore_distance =doc["obstacle_ignore_distance"].GetFloat();
    config.obstacle_collision_cost = doc["obstacle_collision_cost"].GetInt();
    config.historical_l_cost = doc["historical_l_cost"].GetInt();
    config.path_l_cost = doc["path_l_cost"].GetFloat();
    config.path_dl_cost = doc["path_dl_cost"].GetInt();
    config.path_ddl_cost = doc["path_ddl_cost"].GetInt();
    config.path_l_cost_param_l0 = doc["path_l_cost_param_l0"].GetFloat();
    config.path_l_cost_param_b = doc["path_l_cost_param_b"].GetFloat();
    config.path_l_cost_param_k = doc["path_l_cost_param_k"].GetFloat();
    config.path_out_lane_cost = doc["path_out_lane_cost"].GetInt();
    config.path_end_l_cost = doc["path_end_l_cost"].GetInt();
    return ;
}