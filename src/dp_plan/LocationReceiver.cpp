//
// Created by jydragon on 18-7-26.
//

#include <fstream>

#include "LocationReceiver.h"
#include "common.h"

void LocationReceiver::LocationCallback(const iau_ros_msgs::LocationPtr& map_ptr){
    iau_ros_msgs::Location map= *map_ptr;
    RoadPoint tempp;
    //存储，后面的全去掉
    tempp.x = map_ptr->gau_pos[0];
    tempp.y = map_ptr->gau_pos[1];
    tempp.angle = map_ptr->orientation[2] *PI/180.0;
    //此处需要将坐标转成高斯坐标 然后更新当前点坐标，待添加
    tempp = BasicStruct::GussiantoDikaer(tempp);

    currentLocation.x = tempp.x - m_initialPoint.x;
    currentLocation.y = tempp.y - m_initialPoint.y;
    currentLocation.angle = tempp.angle;
//    try {
//        storageLocation(map);
//    }
//    catch (...)
//    {
//        cout<<"storage Location failed..."<<endl;
//    }
    //cout<<"定位..."<<endl;
}
PathPointxy LocationReceiver::GetLocalPath() {
    //updatelocation();
    if(m_wholereferpath.pps.empty())
        return PathPointxy();
    if(maybelocation.empty())
        return PathPointxy();

    //原来的问题应该在这里。
    if(g_currentLocation.x==0&&g_currentLocation.y==0&&g_currentLocation.angle ==0&&g_currentLocation.k ==0)
        return PathPointxy();

    PathPointxy tempLane;
    //先找路径中离当前点最近的点作为起点
    double Mind=MAXD;
    if(pathIndex==0)
    {
        for(int p=0;p<m_wholereferpath.pps.size();++p)
        {
            double dis=BasicStruct::Distance(m_wholereferpath.pps[p],g_currentLocation);
            if(dis<Mind)
            {
                pathIndex =p;
                Mind = dis;
            }
        }
    }
    //cout<<"找到最近的定位点"<<endl;
    int i=pathIndex;
    bool flag_=1;
    double lengtaa = 0.0;
    for(;i<m_wholereferpath.pps.size(); ++i)
    {
        if(i>=m_wholereferpath.pps.size())
            return PathPointxy();
        double angle1 =BasicStruct::AngleNormalnize1(atan2(m_wholereferpath.pps[i].y - g_currentLocation.y, m_wholereferpath.pps[i].x - g_currentLocation.x));
        double angle2 = g_currentLocation.angle;
        double cosangle1 = cos(abs(angle2 - angle1));//保证抛点在车辆前方
        double length=0;
        if(cosangle1>0)
        {
            double dis=BasicStruct::Distance(m_wholereferpath.pps[i],g_currentLocation);
            length+=dis;
            if(flag_){
                pathIndex=i;
                flag_=0;
            }
            tempLane.pps.push_back(m_wholereferpath.pps[i]);
            if(length>80) {
                break;
            }
        }
        lengtaa = length;
    }
    //cout<<"局部路径获取完成，大小："<<tempLane.pps.size()<<endl;
    if(tempLane.pps.empty()||lengtaa<30)
        return PathPointxy();
    else
        //m_referLane= tempLane;
    return tempLane;
}
bool LocationReceiver::readPath(){
    ifstream ifile("/home/z/下载/latticePlan-master/src/testmap.txt");
    if(!ifile){
        cout<<"读取文件失败"<<endl;
        return false;
    }
    RoadPoint ptemp;
    ifile >> ptemp.x >> ptemp.y >> ptemp.angle;
    //此处需要转换成高斯坐标 角度已经转换过了
    m_initialPoint = BasicStruct::LongLattoGussian(ptemp);//坐标初始位置
    RoadPoint last=m_initialPoint;
    while(!ifile.eof()) {
        ifile >> ptemp.x >> ptemp.y >> ptemp.angle;
        //此处需要转换成高斯坐标 角度已经转换过了
        ptemp = BasicStruct::LongLattoGussian(ptemp);
        RoadPoint localpoint;
        localpoint.x = ptemp.x - m_initialPoint.x;
        localpoint.y = ptemp.y - m_initialPoint.y;
        localpoint.angle = ptemp.angle;
        if (BasicStruct::Distance_2(localpoint, last) > 0.01) {
            m_wholereferpath.pps.push_back(localpoint);
            last =localpoint;
        }
    }
    cout<<"读取路径成功"<<endl;
    ifile.close();
    return true;
}

bool LocationReceiver::updatelocation(double time_lidar,RoadPoint &newLocation) {
    //if(maybelocation.empty())
      //  return false;
    //double mindiff = _num_MaxBuffer * 0.01;

    //int  index =0;
//    for(int listiter=0;listiter<maybelocation.size();++listiter)
//    {
//        double xx=fabs(maybelocation[listiter].header.stamp.toSec()-time_lidar);
//        if(xx<mindiff)
//        {
//            index=listiter;
//            mindiff = xx;
//        }
//    }
    //cout<<"最小时间差："<<mindiff<<endl;
    //if(index ==-1)
      //  return false;
    //auto map_ptr =maybelocation[index];
//    RoadPoint tempp;
//    //存储，后面的全去掉
//    tempp.x = map_ptr.gau_pos[0];
//    tempp.y = map_ptr.gau_pos[1];
//    tempp.angle = map_ptr.orientation[2] *PI/180.0;
//    //此处需要将坐标转成高斯坐标 然后更新当前点坐标，待添加
//    tempp = BasicStruct::GussiantoDikaer(tempp);
//
//    newLocation.x = tempp.x - m_initialPoint.x;
//    newLocation.y = tempp.y - m_initialPoint.y;
//    newLocation.angle = tempp.angle;
    //cout << "定位stamp:"<<map_ptr.header.stamp << endl;
    return true;
}

bool LocationReceiver::storageLocation(iau_ros_msgs::Location map) {
    {
        boost::mutex::scoped_lock lock (_mutex_poseVec);
        maybelocation.push_back(map);
        //cout<<"存储"<<endl;
        //int _num_total=maybelocation.size();
        while(maybelocation.size() > _num_MaxBuffer && !maybelocation.empty()) {
            maybelocation.pop_front();
            //cout<<"删除点"<<endl;
        }
        lock.unlock();
    }
    return true;
}
void LocationReceiver::MapCallback(const iau_ros_msgs::MapPtr& map_ptr){//该函数是为了记录地图写的
    //地图当前所在的车道
    const iau_ros_msgs::Map map = *map_ptr;
    m_curindex = map.curindex;
    //地图需要到的车道
    m_maptargetindex = map.targetindex;
    vector<PathPointxy> LaneInfo;
    for (auto& lane : map.road) {
        PathPointxy lp;
        for (auto& pt : lane.points) {
            if (pt.x != -1000&& pt.x != -100000) {
                RoadPoint rp;
                rp.x = pt.x;
                rp.y = pt.y;
                rp.angle = pt.yaw * PI / 180.0;

                RoadPoint di = BasicStruct::GussiantoDikaer(rp);
                RoadPoint re;
                //todo 这里减掉initialPoint的目的是让坐标的值变得小点
                re.x = di.x - m_initialPoint.x;
                re.y = di.y - m_initialPoint.y;
                re.angle = di.angle;
                lp.pps.push_back(re);
            }
        }
		LaneInfo.push_back(lp);
    }
    auto lastMapTime = map.timestamp.toSec();
    m_Lane = LaneInfo;
    cout<<"地图"<<endl;
}

void LocationReceiver::VelocityCallback(const iau_ros_msgs::VelocityPtr velocity_ptr) {
    m_velocity = velocity_ptr->velocity;
}
