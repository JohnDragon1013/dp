//
// Created by jydragon on 18-7-4.
//

#ifndef RRTPLAN_BASICSTRUCT_H
#define RRTPLAN_BASICSTRUCT_H

#include <vector>
#include <cmath>
using namespace std;
struct RoadPoint{
    double x;
    double y;
    double angle;
    double k;
    double cost;
    RoadPoint(){
        x=0;
        y=0;
        angle=0;
        k=0;
        cost=0;
    }
    RoadPoint(double _x,double _y,double _angle,double _k): x(_x), y(_y),angle(_angle), k(_k) ,cost(0){}
};
class BasicStruct {
public:
    BasicStruct() = delete;
    template <class T1,class T2>
    static double Distance(T1 m1,T2 m2)
    {
        return sqrt((m1.x-m2.x)*(m1.x-m2.x)+(m1.y-m2.y)*(m1.y-m2.y));
    }
    template <class T1,class T2>
    static double Distance_2(T1 m1,T2 m2)
    {
        return ((m1.x-m2.x)*(m1.x-m2.x)+(m1.y-m2.y)*(m1.y-m2.y));
    }
    static double AngleNormalnize1(double phi);//0到2pi
    static RoadPoint LongLattoGussian(RoadPoint LLA);//angle输入是角度
    static RoadPoint GussiantoDikaer(RoadPoint org);
    static int WorldtoMap_1(RoadPoint org, double xIn, double yIn, double &xOut, double &yOut);
    static int WorldtoMap(RoadPoint org,double xIn,double yIn,double &xOut,double &yOut);
};
#define PI 3.1415926 //圆周率
#define eps1 2.2204e-016     //一个极小值
#define inf 10000000   //一个很大值
#define MAXD 100000000  //一般用于求最小值的函数中
#define NODESLENGTH 1
#define GRID_I 150          //网格的数量
#define GRID_J 200
#define GRID_WIDTH 0.2		//网格的宽度 0.2米
#define GRID_WIDTH_X 0.2	//网格的x宽度
#define GRID_WIDTH_Y 0.2    //网格的y宽度

struct PathPointxy{
    PathPointxy():length(0),dis(0),obssize(0){};
    std::vector<RoadPoint> pps;
    double length;
    double dis;
    double obssize;
};

struct Car//车结构体
{
    RoadPoint Position;//位置为车后轮轴中心的坐标

    double rearx;//后轮的x和y
    double reary;
    double phi;  //车辆航向角

    double frontx;//前轮的x和y
    double fronty;

    double theta;   //前轮转角
    double length;//车长
    double width;//车宽

    double L;//前后轮轴距
    double RtoT;//后轮距离车尾的距离

};
struct carStatus{
    carStatus():speed(-1),angle(-1),gear(-1){};
    float speed;
    float angle;
    int gear;
};
extern PathPointxy g_LastPath;
extern RoadPoint g_currentLocation;
extern PathPointxy g_LocalReferPath;
extern double fns[];
extern double fds[];
extern double gns[];
extern double gds[];
#endif //RRTPLAN_BASICSTRUCT_H
