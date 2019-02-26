//
// Created by jydragon on 18-7-26.
//

#include <fstream>

#include "LocationReceiver.h"
void LocationReceiver::LocationCallback(const hdmap_msgs::gpfpdPtr& map_ptr){
    hdmap_msgs::gpfpd map= *map_ptr;
    try {
        storageLocation(map);
    }
    catch (...)
    {
        cout<<"storage Location failed..."<<endl;
    }
    //cout<<"定位..."<<endl;
}
PathPointXY LocationReceiver::GetLocalPath() {
    //updatelocation();
    if(m_wholereferpath.pps.empty())
        return PathPointXY();
//    if(maybelocation.empty())
//        return PathPointXY();

    //原来的问题应该在这里。
    if(g_currentLocation.x==0&&g_currentLocation.y==0&&g_currentLocation.angle ==0&&g_currentLocation.k ==0)
        return PathPointXY();

    PathPointXY tempLane;
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
            return PathPointXY();
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
//            RoadPoint org = g_currentLocation;
//            org.angle = BasicStruct::AngleNormalnize1(PI/2.0-g_currentLocation.angle);
//            double xout,yout;
//            BasicStruct::WorldtoMap(org,m_wholereferpath.pps[i].x,m_wholereferpath.pps[i].y,xout,yout);
////            double angle11=BasicStruct::AngleNormalnize1(org.angle + m_wholereferpath.pps[i].angle);
            tempLane.pps.emplace_back(m_wholereferpath.pps[i].x,m_wholereferpath.pps[i].y,m_wholereferpath.pps[i].angle,0.0);
            if(length>80) {
                break;
            }
        }
        lengtaa = length;
    }
    cout<<"局部路径获取完成，大小："<<tempLane.pps.size()<<endl;
    if(tempLane.pps.empty()||lengtaa<30)
        return PathPointXY();
    else
        //m_referLane= tempLane;
    return tempLane;
}
bool LocationReceiver::readPath(){
    ifstream ifile("/home/z/文档/latticePlan-dev/src/testmap.txt");
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

//    if(maybelocation.empty())
//        return false;
    double mindiff = _num_MaxBuffer * 0.01;

//    int  index =-1;
//    for(int listiter=0;listiter<maybelocation.size();++listiter)
//    {
//        double xx=fabs(maybelocation[listiter].header.stamp.toSec()-time_lidar);
//        if(xx<mindiff)
//        {
//            index=listiter;
//            mindiff = xx;
//        }
//    }
//    cout</<"最小时间差："<<mindiff<<endl;
//    if(index ==-1)
//        index =0;//return false; TODO
//    auto map_ptr =maybelocation[0];
    RoadPoint tempp;
    //存储，后面的全去掉 116.1862667	39.8697516	327.764
    tempp.x = 116.1862667;//map_ptr.dLongitude;
    tempp.y = 39.8697516;//map_ptr.dLattitude;
    tempp.angle = 327.764;//map_ptr.dHeading;
    //此处需要将坐标转成高斯坐标 然后更新当前点坐标，待添加
    tempp = BasicStruct::LongLattoGussian(tempp);
    newLocation.x = tempp.x - m_initialPoint.x;
    newLocation.y = tempp.y - m_initialPoint.y;
    newLocation.angle = tempp.angle;
//    cout << "定位stamp:"<<map_ptr.header.stamp << endl;
    return true;
}

bool LocationReceiver::storageLocation(hdmap_msgs::gpfpd map) {
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
void LocationReceiver::MapCallback(const hdmap_msgs::gpfpdPtr& map_ptr){//该函数是为了记录地图写的
    ROS_INFO("received MAP![%f]",map_ptr->dLongitude);
    //将rosbag中的路径，完整记录下来。存入文件
    //iput<<setprecision(12)<<(map_ptr->dLongitude)<<"\t"<<setprecision(12)<<map_ptr->dLattitude<<"\t"<<map_ptr->dHeading<<endl;
    cout<<"地图"<<endl;
}