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
boost::mutex _mutex_update;
ros::Time _time_update;
iau_ros_msgs::GridPtr _LidarMsg;
bool flag_updateLidar=false;
ros::Time _time_register;
void send4Draw(){
    //发送路径信息
    //路径信息
    visualization_msgs::Marker line_path, obsCloud, points_refer;
    visualization_msgs::Marker crashPoint;
    crashPoint.header.frame_id = points_refer.header.frame_id = obsCloud.header.frame_id = line_path.header.frame_id = "velodyne";
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
    obsCloud.scale.x = 0.3;
    obsCloud.scale.y = 0.3;
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
    //其他路径
    pcl::PointCloud<pcl::PointXYZ> other_Path;
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
    ROS_INFO("begin to send path");
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
    //将参考路径转换为局部坐标
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
    }

    vector<unsigned char> griddata = obs.GetGridObs();
    if (griddata.size() > 0) {
        //cloud_obs.width = 600;//ExcutablePath.pps.size();
        //cloud_obs.height = 1000;
        //cloud_obs.points.resize(cloud_obs.width * cloud_obs.height);

        for (int j = 0; j < 1000; ++j) {
            for (int k = 0; k < 600; ++k) {
                geometry_msgs::Point temp;
                if (griddata[j * 600 + k] >= 1) {
                    temp.x = j * GRID_WIDTH - 100;//平移100米
                    temp.y = (k * GRID_WIDTH - 60);//平移60米
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
                pcl::PointXYZ p;
                p.x = yout;
                p.y = -xout;
                p.z = 0;
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
    pcl_pubOtherPath.publish(otherPath_output);
    pcl_pubReferPath.publish(reference_output);
    cout << "发送完成。。。" << endl;

}
//ofstream iput("/home/jydragon/catkin_ws/src/test/src/path.txt");
void LidarCallback(const iau_ros_msgs::GridPtr& cloud_ptr){
    //cout << "激光stamp:" <<cloud_ptr->header.stamp<< endl;
    {
        boost::mutex::scoped_lock lock(_mutex_update);
        _LidarMsg = cloud_ptr;
        flag_updateLidar = true;
        //_time_update = cloud_ptr->header.stamp;
        //obs.SetGridObsInfo(cloud_ptr);

        //ROS_INFO("rostopic update cloud \n");

        _mutex_update.unlock();
    }
    //Location_receiver.setLidarTime(cloud_ptr->header.stamp.toSec());
}

bool XY2SL(const PathPointxy referxy,common::ReferenceLine &referenceLine){
    if(referxy.pps.empty()) {
        ROS_WARN("Reference lane is empty!");
        return false;
    }
    double sum_s =0.0;
    common::ReferencePoint lastp;// =referxy.pps[0];
    lastp.x=referxy.pps[0].x;
    lastp.y=referxy.pps[0].y;
    lastp.heading =referxy.pps[0].angle;
    lastp.s =0.0;
    lastp.l=0.0;
    lastp.kappa_=0.0;
    lastp.dkappa_=0.0;
    referenceLine.reference_points_.push_back(lastp);
    for(int rexy=1;rexy<referxy.pps.size();++rexy)
    {
        common::ReferencePoint reSL;
        double dis =BasicStruct::Distance(referxy.pps[rexy],lastp);
        sum_s+=dis;
        reSL.s =sum_s;
        reSL.l =0.0;
        reSL.x =referxy.pps[rexy].x;
        reSL.y =referxy.pps[rexy].y;
        reSL.heading = referxy.pps[rexy].angle;
        reSL.kappa_ =0.0;//(referxy.pps[rexy].angle-lastp.heading)/dis;
        reSL.dkappa_ =0.0;//(reSL.kappa_-lastp.kappa_)/dis;
        lastp =reSL;//ferxy.pps[rexy];
        referenceLine.reference_points_.push_back(reSL);
    }
    referenceLine.Length =sum_s;
    if(referenceLine.reference_points_.empty())
        return false;
    return true;
}
bool map_invalid(vector<PathPointxy> lanes,int curin,int targetin){
    if(lanes.empty()|| curin<0 || targetin <0 || curin >= lanes.size() || targetin >= lanes.size())
        return true;
    else
        return false;
}
int curIndex;
int targetIndex;
int velocity;
common::DpPolyPathConfig config;
bool beginPlan(){
    g_currentLocation = Location_receiver.GetCurrentLocation();// np;
    try {
        wholeReferLane = Location_receiver.GetMap(curIndex,targetIndex,velocity);//Location_receiver.GetLocalPath();
    }
    catch(...)
    {
        ROS_WARN("referLane update failed... size::[%d]",wholeReferLane.size());
        return false;
    }
    if(map_invalid(wholeReferLane,curIndex,targetIndex))
    {
        ROS_WARN("map invalid!");
        return false;
    }
    referLane = wholeReferLane[targetIndex];
    common::TrajectoryPoint startPo(velocity,0.0);
        //要将xy转换为sl
    common::ReferenceLine reference_line_info;
    if(XY2SL(referLane,reference_line_info)) {
        DPGraphPlan dpp(config, referLane, startPo, reference_line_info, obs ,last_path_);
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
void reSolveRosMsg(const iau_ros_msgs::GridPtr& rosMsg)
{
//_time_register = ros::Time::now();
    //if()
    obs.SetGridObsInfo(rosMsg);
}
void Process(){
    while(ros::ok())
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        {
            //获取ros消息
            boost::mutex::scoped_lock lock(_mutex_update);
            if (!flag_updateLidar  )
                //||fabs(_time_register.toSec() - _time_update.toSec()) < 0.01)
                //continue;
            try {
                //reSolveRosMsg(_LidarMsg);
            }
            catch (...){
                ROS_WARN("_LidarMsg is null...");
                continue;
            }
            lock.unlock();
            flag_updateLidar =false;
        }
        //cout<<"开始规划"<<endl;
        bool flag_failed = beginPlan();
        //plan finished ,storage the data.
        //Location_receiver.logfile();
        //cout << "完成规划" << endl;
        //send4follow(flag_failed);
        send4Draw();
        usleep(1e4);
    }
}
int main(int argc, char **argv)
{
    //common::DpPolyPathConfig config;
    readconfig(config);
    Location_receiver.readPath();//读取一次路径信息
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
    ros::Subscriber sub_lidar = n.subscribe("/IAU/Grid",1,&LidarCallback);
    //订阅定位数据，gps+imu之后由position发出
    ros::Subscriber sub_Location = n.subscribe("/IAU/Location",10,&LocationReceiver::LocationCallback,&Location_receiver);

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
    _time_register =ros::Time();
    //boost::function0<void> f = boost::bind(&send4Draw,1);
    boost::thread thread_viewer(&Process);
    thread_viewer.detach();
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