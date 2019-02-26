//
// Created by jydragon on 18-7-23.
//

#include "Obstacle.h"

/**
 * for test //not used
 */
void Obstacle::SetGridObsInfo() {
    GRID_Num =600000;// cloud_ptr->grid_cols*cloud_ptr->grid_rows;
    grid_widthX = 0.2;
    grid_widthY = 0.2;
    grid_xl = 600;//cloud_ptr->grid_cols;//g_planinput.xl;
    grid_xr = 0;//g_planinput.xr;
    grid_yu = 1000;//cloud_ptr->grid_rows;//g_planinput.yu;
    grid_yd = 0;//g_planinput.yd;
    GRID_WidthNum = grid_xl - grid_xr;
    GRID_LengthNum = grid_yu- grid_yd;
    vector<unsigned char> griddata(GRID_Num);
    for (int i=0;i<GRID_Num;i++)
    {
        if(i>360200&&i<361314)
            griddata[i]=1;
        else
            griddata[i]=0;
    }
    m_GridObs = griddata;
}

void Obstacle::SetGridObsInfo(const lidar_perception::obs2segMsgPtr& cloud_ptr)
{
    //栅格信息 待添加 障碍物更新顺序还有点问题，感觉应该先更新障碍物之后再开始规划
    //这里rows是1000，代表车辆前进方向，cols是600，
    //grid_flag中存储顺序为从右向左存储 顺序如下
//    for(unsigned i = 0 ; i < _gridGenerator._grid_flagobs.rows(); i++)
//    {
//        for(unsigned j = 0 ; j < _gridGenerator._grid_flagobs.cols(); j++)
    //int grid_xl,grid_xr,grid_yu,grid_yd;
    GRID_Num = cloud_ptr->grid_cols*cloud_ptr->grid_rows;
    if(GRID_Num!=600000)
        cout<<"cloud_ptr is null "<<endl;
    grid_widthX = 0.2;
    grid_widthY = 0.2;
    grid_xl = cloud_ptr->grid_cols;//g_planinput.xl;
    grid_xr = 0;//g_planinput.xr;
    grid_yu = cloud_ptr->grid_rows;//g_planinput.yu;
    grid_yd = 0;//g_planinput.yd;
    GRID_WidthNum = grid_xl - grid_xr;
    GRID_LengthNum = grid_yu- grid_yd;
    vector<unsigned char> griddata(GRID_Num);
    //此处需要加上去除地面的部分
    //cloud——ptr可能是空的，会引起崩溃。
    //cout<<"开始填充grid"<<endl;
    int gnum1 = cloud_ptr->cloud_flag.size();
    int gnum2 = cloud_ptr->grid_flag.size();
    int countObs=0;
    for (int i=0;i<cloud_ptr->grid_flag.size();i++)
    {
        griddata[i] = cloud_ptr->grid_flag[i];
//        if(i>360200&&i<360314)
//            griddata[i]=1;
//        else
//            griddata[i]=0;
        if(i>=gnum2){
            cout<< "cloud_ptr.size():"<<gnum1<<" "<<gnum2<<"i:"<<i<<endl;
            break;
        }
//        if(cloud_ptr->grid_flag[i]==1)//此处去掉地面的index
//        {
//                griddata[i] = 1;
//            countObs++;
//        }
//        else
//            griddata[i] = 0;//cloud_ptr->grid_flag[i];
    }
    //cout<< "cloud_ptr.size():"<<countObs<<" "<<gnum1<<endl;
    cout<<"结束填充grid"<<endl;
    m_GridObs = griddata;
}

bool Obstacle::DetectGridObs(Car myCar,RoadPoint &collisionpoint)
{
    //障碍物检测的顺序需要变更，栅格遍历方式不同 坐标不同
    if (m_GridObs.size() < 1)
    {
        //cout<<"-----无障碍物信息------"<<endl;
        return false;
    }
    RoadPoint car1=myCar.Position;

    double dx=myCar.Position.x-g_currentLocation.x;
    double dy=myCar.Position.y-g_currentLocation.y;
    double angle11=BasicStruct::AngleNormalnize1(PI/2.0-g_currentLocation.angle);
    double cx= dx*cos(angle11)-dy*sin(angle11);
    double cy= dx*sin(angle11)+dy*cos(angle11);

    double angle=BasicStruct::AngleNormalnize1(myCar.Position.angle+angle11);

    //double angle=myCar.Position.angle-g_CurrentLocation.angle;

    myCar.Position.x=cx;
    myCar.Position.y=cy;
    myCar.Position.angle=angle;

    myCar.rearx=myCar.Position.x;
    myCar.reary=myCar.Position.y;
    myCar.phi=myCar.Position.angle;

    //此处需要注意算出来的x,y是笛卡尔坐标 这里没有问题
    //但激光格网是按照激光坐标系,x为正前方，y为车辆左侧
    RoadPoint LeftRear;
    LeftRear.x=myCar.rearx-myCar.RtoT*cos(myCar.phi)-0.5*myCar.width*sin(myCar.phi);
    LeftRear.y=myCar.reary-myCar.RtoT*sin(myCar.phi)+0.5*myCar.width*cos(myCar.phi);

    unsigned int LRx=LeftRear.x/grid_widthX+GRID_WidthNum/2;
    unsigned int LRy=LeftRear.y/grid_widthY;

    for(double i=0;i<myCar.width;i+=grid_widthX)
        for(double j=0;j<myCar.length;j+=grid_widthY)
        {

            RoadPoint point;

            point.x=i*sin(myCar.phi)+j*cos(myCar.phi)+LeftRear.x;
            //point.x=-point.x;//坐标转换到激光
            point.y=-i*cos(myCar.phi)+j*sin(myCar.phi)+LeftRear.y;
            int grid_X=-(point.x/grid_widthX)+GRID_WidthNum/2;
            //grid_X = -grid_X;
            int grid_Y=point.y/grid_widthY+GRID_LengthNum/2;

            if(grid_X >= grid_xl)
            {
                return 0;
            }
            if(grid_Y <= 500/*grid_yd*/)
            {
                return 0;
            }
            if(grid_X <= grid_xr)
            {
                return 0;
            }
            if (grid_Y >= /*200*/grid_yu)
            {
                return 0;
            }

            if(m_GridObs[grid_Y*GRID_WidthNum+grid_X] == 1 && grid_X < grid_xl && grid_Y > grid_yd
               && grid_X > grid_xr && grid_Y < grid_yu)
            {
                collisionpoint.x =point.x;
                collisionpoint.y =point.y;
                //cout<<"collision:"<<collisionpoint.x<<" "<<collisionpoint.y<<endl;
                return 1;//
            }
        }
    //cout<<"障碍物检测通过"<<endl;
    return 0;//˵��û���ϰ���
}
//多边形与多边形交的障碍物检测
bool Obstacle::DetectObs_Polygon(Car mycar)
{
//    if(m_Obs.size()==0)
//    {
//        return 0;
//    }
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

    //障碍物的四条边
//    for(int i=0;i<m_Obs.size();i++)
    {
        vector<double> vertX2,vertY2;

        vertX2.push_back(m_Obs/*[i]*/.Polygon[0].x);
        vertX2.push_back(m_Obs/*[i]*/.Polygon[1].x);
        vertX2.push_back(m_Obs/*[i]*/.Polygon[2].x);
        vertX2.push_back(m_Obs/*[i]*/.Polygon[3].x);

        vertY2.push_back(m_Obs/*[i]*/.Polygon[0].y);
        vertY2.push_back(m_Obs/*[i]*/.Polygon[1].y);
        vertY2.push_back(m_Obs/*[i]*/.Polygon[2].y);
        vertY2.push_back(m_Obs/*[i]*/.Polygon[3].y);

        if(BasicStruct::PolygonOverPolygon(vertX1,vertY1,vertX2,vertY2)==1)
        {
            return 1;
        }

    }
    ////转化车所占的所有点到格网，并对其做检查
    return 0;//说明没有障碍物

}
