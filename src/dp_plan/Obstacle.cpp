//
// Created by jydragon on 18-7-23.
//

#include "Obstacle.h"

/**
 * for test //not used
 */
//void Obstacle::SetGridObsInfo() {
//    GRID_Num =600000;// cloud_ptr->grid_cols*cloud_ptr->grid_rows;
//    grid_widthX = 0.2;
//    grid_widthY = 0.2;
//    grid_xl = 600;//cloud_ptr->grid_cols;//g_planinput.xl;
//    grid_xr = 0;//g_planinput.xr;
//    grid_yu = 1000;//cloud_ptr->grid_rows;//g_planinput.yu;
//    grid_yd = 0;//g_planinput.yd;
//    GRID_WidthNum = grid_xl - grid_xr;
//    GRID_LengthNum = grid_yu- grid_yd;
//    vector<unsigned char> griddata(GRID_Num);
//    for (int i=0;i<GRID_Num;i++)
//    {
//        if(i>360200&&i<361314)
//            griddata[i]=1;
//        else
//            griddata[i]=0;
//    }
//    m_GridObs = griddata;
//}
void Obstacle::SetVirtualGridObsInfo() {
    GRID_Num = 160000;//cloud_ptr->height*cloud_ptr->width;
    if(GRID_Num<=0) {
        cout << "cloud_ptr is null " << endl;
        return ;
    }
    grid_widthX = 0.2;
    grid_widthY = 0.2;
    grid_xl = 0;//cloud_ptr->width;//g_planinput.xl;
    grid_xr = 400;//g_planinput.xr;
    grid_yu = 400;//cloud_ptr->height;//g_planinput.yu;
    grid_yd = 0;//g_planinput.yd;
    GRID_WidthNum = grid_xr-grid_xl  ;
    GRID_LengthNum = grid_yu- grid_yd;
    vector<unsigned char> griddata(GRID_Num);
    int num=200;
    for (int i=0;i<GRID_Num;i++)
    {
//(i>37550&&i<37584)||(i>52590&&i<52603)/*直线*/
//C shape
//        if((i>41353&&i<41363)||(i>47698&&i<47700))//|| ((i=num*150+85)&&num<400) )
//            griddata[i]=1;
//        else if((i=num*150+85)&&num<400)
//        {
//            griddata[i]=1;
//        }
        //s型形状约束 前方25米，偏左80000+125*400+200=130200  前方10米 80000+50×400+200
        //if((i>130150&&i<130210)||(i>100191&&i<100300))
        //直线障碍物
        if((i>110100&&i<110193)||(i>140200&&i<140250)  )
            griddata[i]=1;
        else
            griddata[i]=0;
        num++;
    }
    //cout<<"结束填充grid"<<endl;
    m_GridObs = griddata;
}

void Obstacle::SetGridObsInfo(const iau_ros_msgs::GridPtr& cloud_ptr)
{
    //栅格信息 待添加 障碍物更新顺序还有点问题，感觉应该先更新障碍物之后再开始规划
    GRID_Num = cloud_ptr->height*cloud_ptr->width;
    if(GRID_Num<=0) {
        cout << "cloud_ptr is null " << endl;
        return ;
    }
    grid_widthX = 0.2;
    grid_widthY = 0.2;
    grid_xl = 0;
    grid_xr = cloud_ptr->width;
    grid_yu = cloud_ptr->height;
    grid_yd = 0;
    GRID_WidthNum = grid_xr-grid_xl  ;
    GRID_LengthNum = grid_yu- grid_yd;
    vector<unsigned char> griddata(GRID_Num);
    //此处需要加上去除地面的部分
    //cloud——ptr可能是空的，会引起崩溃。
    //cout<<"开始填充grid"<<endl;
    for (int i=0;i<cloud_ptr->grid.size();i++)
    {
        griddata[i] = cloud_ptr->grid[i];
//        if(i>360200&&i<360314)
//            griddata[i]=1;
//        else
//            griddata[i]=0;
//        if(i>=gnum2){
//            cout<< "cloud_ptr.size():"<<gnum1<<" "<<gnum2<<"i:"<<i<<endl;
//            break;
//        }
    }
    //cout<<"结束填充grid"<<endl;
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
            int grid_X=(point.x/grid_widthX)+GRID_WidthNum/2;
            //grid_X = -grid_X;
            int grid_Y=point.y/grid_widthY+GRID_LengthNum/2;

            if(grid_X <= grid_xl)
            {
                return 0;
            }
            if(grid_Y <= grid_yd)/*grid_yd*/
            {
                return 0;
            }
            if(grid_X >= grid_xr)
            {
                return 0;
            }
            if (grid_Y >= /*200*/grid_yu)
            {
                return 0;
            }

            if(m_GridObs[grid_Y*GRID_WidthNum+grid_X] == 1 )//&& grid_X < grid_xl && grid_Y > grid_yd
              // && grid_X > grid_xr && grid_Y < grid_yu)
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


