//
// Created by jydragon on 18-7-4.
//

#include "BasicStruct.h"
#include "Function_GaussProjection.h"
#include <cmath>
RoadPoint g_currentLocation(0,0,0,0);
PathPointxy g_LastPath;//上次路径
PathPointxy g_LocalReferPath;
double fns[] = { 0.49999988085884732562,
                 1.3511177791210715095,
                 1.3175407836168659241,
                 1.1861149300293854992,
                 0.7709627298888346769,
                 0.4173874338787963957,
                 0.19044202705272903923,
                 0.06655998896627697537,
                 0.022789258616785717418,
                 0.0040116689358507943804,
                 0.0012192036851249883877 };
double fds[] = { 1.0,
                 2.7022305772400260215,
                 4.2059268151438492767,
                 4.5221882840107715516,
                 3.7240352281630359588,
                 2.4589286254678152943,
                 1.3125491629443702962,
                 0.5997685720120932908,
                 0.20907680750378849485,
                 0.07159621634657901433,
                 0.012602969513793714191,
                 0.0038302423512931250065 };
double gns[] = { 0.50000014392706344801,
                 0.032346434925349128728,
                 0.17619325157863254363,
                 0.038606273170706486252,
                 0.023693692309257725361,
                 0.007092018516845033662,
                 0.0012492123212412087428,
                 0.00044023040894778468486,
                 -8.80266827476172521e-6,
                 -1.4033554916580018648e-8,
                 2.3509221782155474353e-10 };
double gds[] = { 1.0,
                 2.0646987497019598937,
                 2.9109311766948031235,
                 2.6561936751333032911,
                 2.0195563983177268073,
                 1.1167891129189363902,
                 0.57267874755973172715,
                 0.19408481169593070798,
                 0.07634808341431248904,
                 0.011573247407207865977,
                 0.0044099273693067311209,
                 -0.00009070958410429993314 };



//double BasicStruct::Distance_2(RoadPoint m1,RoadPoint m2) //返回的是两个点的欧氏距离的平方
//{
//    return (m1.x-m2.x)*(m1.x-m2.x)+(m1.y-m2.y)*(m1.y-m2.y);
//}
double BasicStruct::AngleNormalnize1(double phi)
{
    while(phi>2.0*PI)phi=phi-2.0*PI;
    while(phi<0)phi=phi+2.0*PI;
    return phi;
}
//将经纬度转成高斯
RoadPoint BasicStruct::LongLattoGussian(RoadPoint LLA)//angle输入是角度 输出为弧度，x轴为起点
{
	LBtoxy point(LLA.x,LLA.y,6);
	RoadPoint OutPoint;
	point.calculateAll();
	double length = 0.42;
	double m_fGaussX = point.getx();
	double m_fGaussY = point.gety();
	double amuzith = LLA.angle;
	m_fGaussX=m_fGaussX-length*sin(amuzith/180.0*3.1415926);
	m_fGaussY=m_fGaussY+length*cos(amuzith/180.0*3.1415926);
	OutPoint.x=m_fGaussY+500000.0;
	OutPoint.y=m_fGaussX;
	OutPoint.angle=AngleNormalnize1(PI/2.0-LLA.angle*PI/180.0);
	return OutPoint;
}
RoadPoint BasicStruct::GussiantoDikaer(RoadPoint org)
{
	RoadPoint target;
	target.x = org.y;
	target.y = org.x;
	target.angle = AngleNormalnize1(PI / 2.0 - org.angle);
	return target;
}
int BasicStruct::WorldtoMap_1(RoadPoint org, double xIn, double yIn, double &xOut, double &yOut)
{
	double dx = 0, dy = 0, dstX = 0, dstY = 0;
	dx = xIn - org.x;
	dy = yIn - org.y;
	dstX = dx*cos(org.angle) + dy*sin(org.angle);
	dstY = -dx*sin(org.angle) + dy*cos(org.angle);
	xOut = dstX;
	yOut = dstY;
	return -1;
}
int BasicStruct::WorldtoMap(RoadPoint org,double xIn,double yIn,double &xOut,double &yOut)
{
	double dx=0,dy=0,dstX=0,dstY=0;
	dx=xIn-org.x;
	dy=yIn-org.y;
	dstX=dx*cos(org.angle)-dy*sin(org.angle);
	dstY=dx*sin(org.angle)+dy*cos(org.angle);
	xOut=dstX;
	yOut=dstY;
	return -1;
}