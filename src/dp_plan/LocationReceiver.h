//
// Created by jydragon on 18-7-26.
//
#include "ros/ros.h"
#include "../msgHeader/gpfpd.h"
#include "../BasicStruct.h"
#include <boost/thread/mutex.hpp>
#include <deque>

#ifndef LATTICEPLAN_LOCATIONRECEIVER_H
#define LATTICEPLAN_LOCATIONRECEIVER_H

#define _num_MaxBuffer 300
class LocationReceiver {
public:
    LocationReceiver():pathIndex(0){};
    ~LocationReceiver()= default;
    void LocationCallback(const hdmap_msgs::gpfpdPtr& map_ptr);
    void MapCallback(const hdmap_msgs::gpfpdPtr& map_ptr);//该函数是为了记录地图写的
    PathPointXY GetLocalPath();
    bool readPath();
    bool updatelocation(double time_lidar,RoadPoint &newLocation);
    //void setLidarTime(double x){lidar_time =x;};
    bool storageLocation(hdmap_msgs::gpfpd map);
private:
    RoadPoint m_initialPoint;
    PathPointXY m_wholereferpath;
    PathPointXY m_referLane;
    std::deque<hdmap_msgs::gpfpd> maybelocation;
    int pathIndex;
    //double lidar_time;
    boost::mutex _mutex_poseVec;
};


#endif //LATTICEPLAN_LOCATIONRECEIVER_H
