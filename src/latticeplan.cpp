#include "ros/ros.h"
#include "std_msgs/String.h"
#include "msgHeader/obs2segMsg.h"


//#include "dp_plan/Obstacle.h"
#include "dp_st_speed/st_path_obstacle_test.h"
#include "dp_plan/LocationReceiver.h"
#include "dp_plan/DPGraphPlan.h"
#include "dp_st_speed/sp_st_optimizer.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include<iomanip>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <pthread.h>
common::DiscretizedPath ExcutablePath;
PathPointXY wholereferpath;
PathPointXY referLane;
common::DiscretizedTrajectory trajectory_;
Obstacle obs;
vector<PathObstacle> pathobstacles_;
LocationReceiver Location_receiver;

RoadPoint initialPoint;
RoadPoint lastLocation;
RoadPoint collisionPoint;
vector<PathPointXY> path4draw;
common::FrenetFramePath last_path_;
void send4Draw();
void beginPlan();
void drawCar();
void translateLocalpath(PathData &pathData_);
ros::Publisher pcl_pubPath ;
ros::Publisher pcl_pubOtherPath ;
ros::Publisher pcl_pubReferPath ;
ros::Publisher pub_dynamicObs ;
boost::mutex _mutex_update;
ros::Time _time_update;
ros::Time t1;
lidar_perception::obs2segMsgPtr _LidarMsg;
bool flag_updateLidar=false;
//nav_msgs::Odometry _odo_cloud;
ros::Time _time_register;
bool flag_firstdraw = true;
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
    line_path.scale.x = 0.3;
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
    for (size_t i = 0; i < ExcutablePath.size(); ++i) {
//        double xout, yout;
//        RoadPoint org = g_currentLocation;
//        org.angle = BasicStruct::AngleNormalnize1(PI / 2.0 - g_currentLocation.angle);
//        BasicStruct::WorldtoMap(org, ExcutablePath[i].x, ExcutablePath[i].y, xout, yout);
        geometry_msgs::Point p;
        p.x = ExcutablePath[i].y;//yout;
        p.y = -ExcutablePath[i].x;//-xout;
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
    ros::Time t2 = ros::Time::now();
    auto deltaT = t2.toSec()-t1.toSec();
    visualization_msgs::Marker obs_cube1,obs_cube2;
    obs_cube2.header.frame_id=obs_cube1.header.frame_id = "velodyne";
    obs_cube2.header.stamp=obs_cube1.header.stamp = ros::Time::now();
    obs_cube2.ns=obs_cube1.ns = "Plan_Data";
    obs_cube1.id = 1;obs_cube2.id =2;
    obs_cube2.type = obs_cube1.type = visualization_msgs::Marker::LINE_STRIP;
    obs_cube2.scale.x = obs_cube1.scale.x = 0.3;
    obs_cube2.color.r =obs_cube1.color.r = 1.0;
    obs_cube2.color.b =obs_cube1.color.b = 1.0;
    obs_cube2.color.g =obs_cube1.color.g = 19.0;
    obs_cube2.color.a =obs_cube1.color.a = 20.0;
    int n =0;
    for(int obs_ = 0;obs_<pathobstacles_.size();++obs_)
    {
        vector<common::PathPoint> D_obs = pathobstacles_[obs_].location4drae(deltaT);
        if(obs_ == 0){
            for(int j=0;j<D_obs.size();++j)
            {
                geometry_msgs::Point po;
                po.x = D_obs[j].y;
                po.y = -D_obs[j].x;
                obs_cube1.points.emplace_back(po);
            }
            if(!D_obs.empty()) {
                geometry_msgs::Point po;
                po.x = D_obs[0].y;
                po.y = -D_obs[0].x;
                obs_cube1.points.emplace_back(po);
            }
            obs_cube1.text ="speed";
        } else
        {
            for(int j=0;j<D_obs.size();++j)
            {
                geometry_msgs::Point po;
                po.x = D_obs[j].y;
                po.y = -D_obs[j].x;
                obs_cube2.points.emplace_back(po);
            }
            if(!D_obs.empty()) {
                geometry_msgs::Point po;
                po.x = D_obs[0].y;
                po.y = -D_obs[0].x;
                obs_cube2.points.emplace_back(po);
            }
        }

    }
    drawCar();
//    vector<PathObstacle> path_obs_=
//    vector<unsigned char> griddata = obs.GetGridObs();
//    //obs
//    if (griddata.size() > 0) {
//        //cloud_obs.width = 600;//ExcutablePath.pps.size();
//        //cloud_obs.height = 1000;
//        //cloud_obs.points.resize(cloud_obs.width * cloud_obs.height);
//
//        for (int j = 0; j < 1000; ++j) {
//            for (int k = 0; k < 600; ++k) {
//                geometry_msgs::Point temp;
//                if (griddata[j * 600 + k] >= 1) {
//                    temp.x = j * GRID_WIDTH - 100;//平移100米
//                    temp.y = (k * GRID_WIDTH - 60);//平移60米
//                    temp.z = 0;
//                    obsCloud.points.push_back(temp);
//
//                } else {}
//            }
//        }
//
//    }

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
    pub_dynamicObs.publish(obs_cube1);
    pub_dynamicObs.publish(obs_cube2);
    pcl_pubOtherPath.publish(otherPath_output);
    pcl_pubReferPath.publish(reference_output);
    cout << "发送完成。。。" << endl;

}
//ofstream iput("/home/jydragon/catkin_ws/src/test/src/path.txt");
void LidarCallback(const lidar_perception::obs2segMsgPtr& cloud_ptr){
    cout << "激光stamp:" <<cloud_ptr->header.stamp<< endl;
    {
        boost::mutex::scoped_lock lock(_mutex_update);
        _LidarMsg = cloud_ptr;
        flag_updateLidar = true;
        _time_update = cloud_ptr->header.stamp;
        //obs.SetGridObsInfo(cloud_ptr);

        //ROS_INFO("rostopic update cloud \n");

        _mutex_update.unlock();
    }

    //Location_receiver.setLidarTime(cloud_ptr->header.stamp.toSec());

    //send4Draw();
}
struct timeval t_star;// t2;

void drawCar(){
    visualization_msgs::Marker car_cube,text_view;
    text_view.header.frame_id =car_cube.header.frame_id = "velodyne";
    text_view.header.stamp =car_cube.header.stamp = ros::Time::now();
    text_view.ns=car_cube.ns = "Plan_Data";
    car_cube.id=3;text_view.id=4;
    car_cube.type =visualization_msgs::Marker::CUBE;
    text_view.type =visualization_msgs::Marker::TEXT_VIEW_FACING;
//    lastt = ros::Time::now().nsec;
//    struct timeval t2;
//    gettimeofday(&t2, NULL);
    auto tend = ros::Time::now();
    auto sec = tend.toSec()- t1.toSec();//(t2.tv_sec - t_star.tv_sec) * 1000000 + t2.tv_usec - t_star.tv_usec;//微秒
    //double sec = deltaT/1000000;
    for(const auto &tra:trajectory_)
    {
        if(sec<=tra.relative_time)
        {
            car_cube.pose.position.x = tra.path_point.y;
            car_cube.pose.position.y = -tra.path_point.x;
            car_cube.pose.position.z = 0;//tra.path_point.x;
            car_cube.pose.orientation.x = 0.0;
            car_cube.pose.orientation.y = 0.0;
            car_cube.pose.orientation.z = tra.path_point.theta;
            car_cube.pose.orientation.w = 0.0;

            text_view.pose.position.x = tra.path_point.y;
            text_view.pose.position.y = -tra.path_point.x-12;
            text_view.pose.position.z = 0;//tra.path_point.x;
            ostringstream str;
            str<<"velocity:"<<tra.v;
            text_view.text = str.str();
            break;
        }
    }
    if(!trajectory_.empty()&&sec>trajectory_.back().relative_time)
    {
        car_cube.pose.position.x = trajectory_.back().path_point.y;
        car_cube.pose.position.y = -trajectory_.back().path_point.x;
        car_cube.pose.position.z = 0;//tra.path_point.x;
        car_cube.pose.orientation.z = trajectory_.back().path_point.theta;
    }
    //car_cube.pose.position.x =

    car_cube.color.g = 1.0;car_cube.color.r = 0.1;
    car_cube.color.b = 0.1;
    car_cube.color.a = 1.0;
    car_cube.scale.x = 4.2;
    car_cube.scale.y = 2.2;
    car_cube.scale.z = 1.0;
    text_view.scale.z =4.0;
    text_view.color.g = 1.0;text_view.color.r = 0.1;
    text_view.color.b = 0.1;
    text_view.color.a = 1.0;

    pub_dynamicObs.publish(car_cube);
    pub_dynamicObs.publish(text_view);
}
bool XY2SL(const PathPointXY referxy,common::ReferenceLine &referenceLine){
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
void beginPlan(){
//    try {
//        RoadPoint np;
////        bool flag_=Location_receiver.updatelocation(_time_register.toSec(),np);
//        if(!flag_)
//            ROS_WARN("Update Location failed....");
//        g_currentLocation = np;
//    }
//    catch (...){
//        ROS_WARN("Update Location Failed and crashed.");
//        return ;
//    }
    try {
        referLane = Location_receiver.GetLocalPath();
    }
    catch(...)
    {
        ROS_WARN("referLane update failed... size::[%d]",referLane.pps.size());
        return;
    }
        //要将xy转换为sl
    common::ReferenceLine reference_line_info;
    if(XY2SL(referLane,reference_line_info)) {
        common::DpPolyPathConfig config;
        DPGraphPlan dpp(config, referLane, reference_line_info, obs ,last_path_);
        //cout << "start Plan" << endl;
        PathData ep = dpp.Getfinalpath(last_path_);
        translateLocalpath(ep);
        DpStSpeedOptimizer stSpeedOptimizer(ep);
        SpeedData speedData_;//the final speed data(output)
        common::TrajectoryPoint initial_point; //which should include position,velocity,accelarate,
        initial_point.path_point = common::PathPoint(g_currentLocation.x,g_currentLocation.y,g_currentLocation.angle,g_currentLocation.k);
        //need to assign velocity and accelaration
        initial_point.v = 5.0;//cur_velocity;
//        gettimeofday(&t1,NULL);

        stSpeedOptimizer.Process(ep,initial_point,pathobstacles_,&speedData_);
        double relative_time=0.0; double start_s = initial_point.path_point.s;
        stSpeedOptimizer.CombinePathAndSpeedProfile(relative_time,start_s,ep,&speedData_,&trajectory_);
        //stSpeedOptimizer.
        path4draw = dpp.GetAllPath();
        //这里需要返回两条路径 然后再进行速度文件的计算，最后combine两种数据，分别每个路径点对应
        //DpStSpeedOptimizer ();
        if(ep.discretized_path().empty())
            ROS_WARN("Generating path failed");
        else
            ROS_INFO("Generating path success");
        ExcutablePath =ep.discretized_path();
    }
    else{
        common::DiscretizedPath ep;
        ExcutablePath =ep;//置为空
        vector<PathPointXY> p4;
        path4draw = p4;
        ROS_WARN("can't translate reference lane from XY to SL");
    }

}
void reSolveRosMsg(const lidar_perception::obs2segMsgPtr& rosMsg)
{
    _time_register = rosMsg->header.stamp;
    //if()
    obs.SetGridObsInfo(rosMsg);
}
void Process(){
    while(ros::ok())
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        {
            //获取ros消息
//            boost::mutex::scoped_lock lock(_mutex_update);
//            if (!flag_updateLidar||
//                fabs(_time_register.toSec() - _time_update.toSec()) < 0.01)
//                continue;
            //Location_receiver.updatelocation();
//            try {
//                reSolveRosMsg(_LidarMsg);
//            }
//            catch (...){
//                ROS_WARN("_LidarMsg is null...");
//                continue;
//            }
//            lock.unlock();
            flag_updateLidar =false;
        }
        //cout<<"开始规划"<<endl;
        beginPlan();
        if(flag_firstdraw)
        {
            flag_firstdraw = false;
            t1 = ros::Time::now();
            gettimeofday(&t_star, NULL);
        }
        //cout << "完成规划" << endl;
        send4Draw();
    }
}
void MsgCallback(ros::NodeHandle n)
{
    ROS_INFO("start to receive map and lidar!");
}
void loadnewobstacle(vector<PathObstacle>* path_obs ){
    for(const auto obs:*path_obs)
    {
//        vector
    }
}
void sendThread(){
    while(ros::ok())
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(5));
        send4Draw();
    }
}
int main(int argc, char **argv)
{
    Location_receiver.readPath();//读取一次路径信息
    ros::init(argc, argv, "test_ljy");//0711 测试RRT注掉
    ros::NodeHandle n;//一个node貌似可以接收多个消息
    pcl_pubPath = n.advertise<visualization_msgs::Marker> ("Path_output", 1);
    pcl_pubOtherPath =n.advertise<sensor_msgs::PointCloud2>("OtherPath",1);
    pcl_pubReferPath = n.advertise<sensor_msgs::PointCloud2> ("Reference_Lane",1);
    pub_dynamicObs = n.advertise<visualization_msgs::Marker>("Dynamic_obs",1);

    //dev test ,dynamic obstacle
//    ros::Subscriber sub_lidar = n.subscribe("/lidar_obstacle/obs2segment",1,&LidarCallback);
//    ros::Subscriber sub_map = n.subscribe("/sensor/gpfpd",1,&LocationReceiver::LocationCallback,&Location_receiver);

    bool flag_=Location_receiver.updatelocation(_time_register.toSec(),g_currentLocation);
    _time_update= ros::Time();
    _time_register =ros::Time();

    //boost::function0<void> f = boost::bind(&send4Draw,1);
    boost::thread thread_process(&Process);
    thread_process.detach();
//    boost::thread thread_viewer(&sendThread);
//    thread_viewer.detach();
//    gettimeofday(&t1,NULL);
    std::cout<<"Start Listening!"<<std::endl;
    //MsgCallback(n);
    while (ros::ok())
    {
        usleep(1e3);
        ros::spinOnce();
    }
    //sleep(1);
    return 0;
}
void translateLocalpath(PathData &pathData_){
    if(pathData_.discretized_path().empty())
        return;
    auto ep = pathData_.discretized_path();
    common::DiscretizedPath localpath;
    for (size_t i = 0; i < ep.size(); ++i) {
        double xout, yout;
        RoadPoint org = g_currentLocation;
        org.angle = BasicStruct::AngleNormalnize1(PI / 2.0 - g_currentLocation.angle);
        BasicStruct::WorldtoMap(org, ep[i].x, ep[i].y, xout, yout);
        common::PathPoint p;
        p.x = xout;
        p.y = yout;
        p.theta =BasicStruct::AngleNormalnize1(org.angle+ ep[i].theta);
        p.s = ep[i].s;
//        p.z = 0;
        localpath.push_back(p);
    }
    pathData_.SetDiscretizedPath(localpath);
}
