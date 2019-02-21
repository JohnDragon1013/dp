//
// Created by jydragon on 18-7-26.
//
#include "ros/ros.h"
//#include "../msgHeader/gpfpd.h"
#include "../msgHeader/Location.h"
#include "../msgHeader/Map.h"
#include "../msgHeader/Velocity.h"
#include "../BasicStruct.h"
#include "common.h"
#include <boost/thread/mutex.hpp>
#include <deque>

#ifndef LATTICEPLAN_LOCATIONRECEIVER_H
#define LATTICEPLAN_LOCATIONRECEIVER_H

#define _num_MaxBuffer 300
class LocationReceiver {
public:
    LocationReceiver():pathIndex(0){};
    ~LocationReceiver()= default;
    void LocationCallback(const iau_ros_msgs::LocationPtr& map_ptr);
    void MapCallback(const iau_ros_msgs::MapPtr& map_ptr);//该函数是为了记录地图写的
    void VelocityCallback(const iau_ros_msgs::VelocityPtr velocity_ptr);
    PathPointxy GetLocalPath();
    bool readPath();
    bool updatelocation(double time_lidar,RoadPoint &newLocation);
    //void setLidarTime(double x){lidar_time =x;};
    bool storageLocation(iau_ros_msgs::Location map);

    inline RoadPoint GetCurrentLocation()const{return currentLocation;};
    inline vector<PathPointxy> GetMap(int &curindex,int &targetindex,int &velocity)const{curindex = m_curindex;
    targetindex = m_maptargetindex;
    velocity = m_velocity;
        return m_Lane;};
private:
    RoadPoint m_initialPoint;
    RoadPoint currentLocation;
    PathPointxy m_wholereferpath;
    PathPointxy m_referLane;
    std::deque<iau_ros_msgs::Location> maybelocation;
    int pathIndex;
    //double lidar_time;
    boost::mutex _mutex_poseVec;
    int m_curindex;
    int m_maptargetindex;
    int m_velocity;
    vector<PathPointxy> m_Lane;
};


#endif //LATTICEPLAN_LOCATIONRECEIVER_H
