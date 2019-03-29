//
// Created by jydragon on 18-7-26.
//



#include "LocationReceiver.h"
#include "common.h"
//
#define PASSABLELENGTH 35
void LocationReceiver::LocationCallback(const iau_ros_msgs::LocationPtr& map_ptr){
    iau_ros_msgs::Location map= *map_ptr;
    RoadPoint tempp;
    //存储，后面的全去掉
    tempp.x = map_ptr->gau_pos[0];
    tempp.y = map_ptr->gau_pos[1];
    tempp.angle = map_ptr->orientation[2] ;//*PI/180.0;
    //此处需要将坐标转成高斯坐标 然后更新当前点坐标，待添加
    //tempp = BasicStruct::GussiantoDikaer(tempp);
    if(!flag_initial)
    {
        m_initialPoint = tempp;
        flag_initial = true;
    }
    currentLocation.x = tempp.x - m_initialPoint.x;
    currentLocation.y = tempp.y - m_initialPoint.y;
    currentLocation.angle = tempp.angle;
}
PathPointxy LocationReceiver::GetLocalPath(Obstacle &obs) {
    obs = m_obstacle;
    //updatelocation();
    if(m_wholereferpath.pps.empty())
        return PathPointxy();
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

        //m_referLane= tempLane;
    return tempLane;
}
bool LocationReceiver::readPath(){
    ifstream ifile("/home/z/文档/仿真地图/直线.txt");
    if(!ifile){
        cout<<"读取文件失败"<<endl;
        return false;
    }
    RoadPoint ptemp;
    ifile >> ptemp.x >> ptemp.y >> ptemp.angle;
    //此处需要转换成高斯坐标 角度已经转换过了
    m_initialPoint = ptemp;//BasicStruct::LongLattoGussian(ptemp);//坐标初始位置
    RoadPoint last=m_initialPoint;
    while(!ifile.eof()) {
        ifile >> ptemp.x >> ptemp.y >> ptemp.angle;
        //此处需要转换成高斯坐标 角度已经转换过了
        //ptemp = BasicStruct::LongLattoGussian(ptemp);
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
void LocationReceiver::LidarCallback(const iau_ros_msgs::GridPtr& cloud_ptr){
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
bool LocationReceiver::reSolveRosMsg(/*const iau_ros_msgs::GridPtr& rosMsg*/)
{
    m_obstacle.SetVirtualGridObsInfo();
    return true;
    if(flag_updateLidar)
    {
        boost::mutex::scoped_lock lock(_mutex_update);

        m_obstacle.SetGridObsInfo(_LidarMsg);
        lock.unlock();
        flag_updateLidar=false;
    } else
        return false;
    ChangeLaneDecide();
    return true;
}
void LocationReceiver::MapCallback(const iau_ros_msgs::MapPtr& map_ptr){//该函数是为了记录地图写的
    if(m_initialPoint.x==-11111)// 避免坐标原点未被赋值
        return ;
    //地图当前所在的车道
    const iau_ros_msgs::Map map = *map_ptr;
    m_curindex = map.curindex;
    //地图需要到的车道
    m_maptargetindex = map.targetindex;
    m_TargetLane = m_maptargetindex;
    vector<PathPointxy> LaneInfo;
    for (auto& lane : map.road) {
        PathPointxy lp;
//        RoadPoint lastrp;
//        bool falg =1;
        for (auto& pt : lane.points) {
            if (pt.x != -1000&& pt.x != -100000) {
                RoadPoint rp;
                rp.x = pt.x;
                rp.y = pt.y;
                rp.angle = pt.yaw * PI / 180.0;

                RoadPoint di = BasicStruct::GussiantoDikaer(rp);
                RoadPoint re;
                /// 这里减掉initialPoint的目的是让坐标的值变得小点
                re.x = di.x - m_initialPoint.x;
                re.y = di.y - m_initialPoint.y;
                re.angle = di.angle;
//                if(falg) {
//                    re.k = 0;
//                    falg=0;
//                }
//                else{
//                    double dis = BasicStruct::Distance(re,lastrp);
//                    re.k = (re.angle-lastrp.angle)/dis;
//                }
                lp.pps.push_back(re);
                //lastrp = re;
            }
        }
		LaneInfo.push_back(lp);
    }
    auto lastMapTime = map.timestamp.toSec();
    m_Lane = LaneInfo;
    //cout<<"地图"<<endl;
}

void LocationReceiver::VelocityCallback(const iau_ros_msgs::VelocityPtr velocity_ptr) {
    m_velocity = velocity_ptr->velocity;
    //m_velocity/=3.6;//换成m/s
}

void LocationReceiver::VehicleStatusCallback(const iau_ros_msgs::VehicleStatusPtr vehicleStatusPtr) {
    m_carStatus.speed = vehicleStatusPtr->vehicleSpeed;
    m_carStatus.speed/=3.6;
    m_carStatus.angle = vehicleStatusPtr->steerAngle;
    m_carStatus.gear = vehicleStatusPtr->gear;
}


void LocationReceiver::CollisionTest()
{
    //Plan_Type planType;
    int collisionindex;
    double collisionDis;

    vector<double> lanecollsion;
    for (int i = 0; i < m_Lane.size(); i++)
    {
        //判断车辆与地图发送的车道数据是否存在碰撞
        ObstacleonLane(m_Lane[i], collisionindex,collisionDis);
        lanecollsion.push_back(collisionDis);
    }
    m_lanecollsion = lanecollsion;
    return ;
}
bool LocationReceiver::ObstacleonLane(PathPointxy path, int &collisionindex ,double &collisionDis)
{
    collisionindex = 100;
    collisionDis = 100;
    if (path.pps.size() == 0)
    {
        return 1;
    }
    //车的模型
    Car GridObsDetect;
    GridObsDetect.length = 4.2;//g_ConfigFile.car_length;
    GridObsDetect.RtoT = 0.8;//g_ConfigFile.car_RtoT;
    GridObsDetect.width = 2.0;//g_ConfigFile.car_width_W;
    for (int i = 0; i<path.pps.size(); i++)
    {
        GridObsDetect.Position = path.pps[i];
        GridObsDetect.Position.angle = GridObsDetect.Position.angle;
        RoadPoint collisionpoint;
        //只检测车体前方40m，200×0.2;0.2是激光网格的分辨率，一个格网大小代表20cm×2-cm
        // m_obstacle.SetDetectInfo(200);
        if (m_obstacle.DetectGridObs(GridObsDetect, collisionpoint) == 1)//����ģʽ���ϰ����ж�
        {
            collisionindex = i;
            collisionDis = BasicStruct::Distance(g_currentLocation,path.pps[i]);
            return 1;
        }
    }
    return 0;
}
double LocationReceiver::DisCurrentToPath(PathPointxy path)
{
    double minDis = MAXD;
    for (int i = 0; i<path.pps.size(); i++)
    {
        double Dis = BasicStruct::Distance(path.pps[i], g_currentLocation);
        if (Dis<minDis)
        {
            minDis = Dis;
        }
    }
    return minDis;
}
///判断是否需要换道
void LocationReceiver::ChangeLaneDecide() {
    CollisionTest();
    if(m_lanecollsion.empty())
        return;

    //还需要计算前车距离和时间
    struct timeval t1, t2;
    //gettimeofday(&t1, NULL);

    auto curSpeed = m_carStatus.speed;
    gettimeofday(&t2, NULL);
    auto deltaT = (t2.tv_sec - last_timeStamp.tv_sec) * 1000000 + t2.tv_usec - last_timeStamp.tv_usec;//微秒
    double dt = deltaT/1000000.0;
    double ObsSpeed =curSpeed- ((last_ObsDis - m_lanecollsion[m_curindex])/dt);
    if(ObsSpeed>10) //
        ObsSpeed = 8;
    ObsSpeed= ObsSpeed<0?0:ObsSpeed;

    double safe_Dis = 1.5*curSpeed + (curSpeed*curSpeed - ObsSpeed*ObsSpeed)/(2*0.7*9.8);//安全距离计算方式，详见论文

    last_timeStamp = t2;
    if (m_lanecollsion[m_maptargetindex] >= 15)//safe_Dis)
    {
        //沿当前车道前行 速度按照障碍物速度发送
    }
    else // 搜索可通行车道
    {
        if(m_lanecollsion.size()>=2) {
            //get the other lane
            int otherlane;
            if(m_TargetLane == 0) otherlane = 1;
            else otherlane = 0;
            if (m_lanecollsion[otherlane] >= PASSABLELENGTH)
            {
                m_TargetLane = otherlane;
                //flag_changeLane = 1;
            }
        }

    }
}

/*用来存储接受到数据，到文本文件中。暂定的存储为
 * 时间戳（ros time）
 * 定位
 * map //貌似不用记录
 * 车辆信息
 * 前车距离 （如果有） 前车速度
 * */
void LocationReceiver::logfile() {
    log<<ros::Time().now().toSec()<<"\t" <<currentLocation.x<<"\t"<< currentLocation.y<<"\t"<<currentLocation.angle <<"\t"
       << m_carStatus.speed <<"\t"<<m_carStatus.angle;
    log<<endl;
}