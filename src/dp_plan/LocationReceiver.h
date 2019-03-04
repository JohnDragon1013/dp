//
// Created by jydragon on 18-7-26.
//
#include "ros/ros.h"
//#include "../msgHeader/gpfpd.h"
#include "../msgHeader/Location.h"
#include "../msgHeader/Map.h"
#include "../msgHeader/Velocity.h"
#include "../msgHeader/VehicleStatus.h"
#include "../msgHeader/Grid.h"
#include "../BasicStruct.h"
#include "common.h"
#include "Obstacle.h"
#include <boost/thread/mutex.hpp>
#include <fstream>
#ifndef LATTICEPLAN_LOCATIONRECEIVER_H
#define LATTICEPLAN_LOCATIONRECEIVER_H

#define _num_MaxBuffer 300
class LocationReceiver {
public:
    LocationReceiver():pathIndex(0),m_curindex(-1),m_maptargetindex(-1),m_velocity(-1){
        log.open("logFile.txt");
        m_initialPoint.x=-11111;
    };
    ~LocationReceiver()= default;
    void LocationCallback(const iau_ros_msgs::LocationPtr& map_ptr);
    void MapCallback(const iau_ros_msgs::MapPtr& map_ptr);//该函数是为了记录地图写的
    void LidarCallback(const iau_ros_msgs::GridPtr& cloud_ptr);
    void VelocityCallback(const iau_ros_msgs::VelocityPtr velocity_ptr);
    void VehicleStatusCallback(const iau_ros_msgs::VehicleStatusPtr vehicleStatus_Ptr);
    PathPointxy GetLocalPath();
    bool readPath();
    bool updatelocation(double time_lidar,RoadPoint &newLocation);
    //void setLidarTime(double x){lidar_time =x;};
    bool storageLocation(iau_ros_msgs::Location map);
    void logfile();
    inline RoadPoint GetCurrentLocation()const{return currentLocation;};
    inline vector<PathPointxy> GetMap(int &curindex,int &targetindex,double &velocity,Obstacle &obs)const{curindex = m_curindex;
    targetindex = m_TargetLane;
    velocity = m_carStatus.speed;
    obs=m_obstacle;
        return m_Lane;};
    void ChangeLaneDecide();
    void reSolveRosMsg();

private:
    void CollisionTest();
    bool ObstacleonLane(PathPointxy path, int &collisionindex ,double &collisionDis);

    RoadPoint m_initialPoint;
    RoadPoint currentLocation;
    PathPointxy m_wholereferpath;
    PathPointxy m_referLane;
    //std::deque<iau_ros_msgs::Location> maybelocation;
    int pathIndex;
    //double lidar_time;
    boost::mutex _mutex_poseVec;
    iau_ros_msgs::GridPtr _LidarMsg;
    Obstacle m_obstacle;
    int m_curindex;
    int m_maptargetindex;
    int m_TargetLane;
    double m_velocity;
    vector<PathPointxy> m_Lane;
    carStatus m_carStatus;
    ofstream log;//("logfile");

    bool flag_initial = false;

    vector<double > m_lanecollsion;//存储路径碰撞距离。
    boost::mutex _mutex_update;//更新激光

    struct timeval last_timeStamp;//last time plan thread
    double last_ObsDis;
};


#endif //LATTICEPLAN_LOCATIONRECEIVER_H
