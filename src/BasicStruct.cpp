//
// Created by jydragon on 18-7-4.
//

#include "BasicStruct.h"
#include "Function_GaussProjection.h"
#include <cmath>
RoadPoint g_currentLocation(0,0,0,0);
PathPointXY g_LastPath;//上次路径
PathPointXY g_LocalReferPath;
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
    if(phi>2.0*PI)phi=phi-2.0*PI;
    if(phi<0)phi=phi+2.0*PI;
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
bool BasicStruct::PolygonOverPolygon(vector<double>vertX, vector<double>vertY, vector<double>_vertX, vector<double>_vertY)
{//判断两多边形是否相交
	//判断vert是否有顶点在_vert内
	for(int i=0;i<=(int)vertX.size()-1;i++)
	{
		//看vert[i]是否在_vert内
		if(PointInPolygon(_vertX,_vertY,vertX[i],vertY[i]))
		{
			return true;
		}
	}
	//判断_vert是否有顶点在vert内
	for(int i=0;i<=(int)_vertX.size()-1;i++)
	{
		//看_vert[i]是否在vert内
		if(PointInPolygon(vertX,vertY,_vertX[i],_vertY[i]))
		{
			return true;
		}
	}
	//判断vert和_vert是否有相交边
	int n=(int)vertX.size();
	int _n=(int)_vertX.size();
	for(int i=0;i<=n-1;i++)
	{
		for(int j=0;j<=_n-1;j++)
		{
			//判断边(i,i+1)和边(j,j+1)是否相交
			if(LineSegOverLineSeg(vertX[i],vertY[i],vertX[(i+1)%n],vertY[(i+1)%n],
								  _vertX[j],_vertY[j],_vertX[(j+1)%_n],_vertY[(j+1)%_n]))
			{
				return true;
			}
		}
	}
	return false;
}
bool BasicStruct::PointInPolygon(vector<double>vertX, vector<double>vertY,double x,double y)
{//判断点(x,y)是否在多边形(vertX,vertY)内
	//const double a=0;
	//const long inf=10000;
	const long infi=10000;
	//构造水平向左的射线(x,y)->(xfar,yfar)
	long xfar=-infi;
	long yfar=y;
	double nCross=0;//穿过次数
	int XV=0;//射线穿过的极值数
	vector<double> Hmap;//高度值表，用于记录x坐标比射线小的各点y坐标与射线的大小关系
	Hmap.resize((int)vertX.size());
	//高度值宏
	const int noValue=0;
	const int high=1;
	const int low=2;
	const int even=3;
	//填充HLmap
	int n=(int)vertX.size();
	for(int i=0;i<=n-1;i++){
		if(vertX[i]>=x){
			Hmap[i]=noValue;
		}else{
			if(vertY[i]>y)Hmap[i]=high;
			else if(vertY[i]==y)Hmap[i]=even;
			else Hmap[i]=low;
		}
	}
	//计算nCross
	for(int i=0;i<=n-1;i++){
		if(Hmap[i]!=even&&Hmap[(i+1)%n]!=even){
			//如果边的两个端点都不在射线上，则需要判断是否穿过
			bool cross=LineSegOverLineSeg(x,y,xfar,yfar,
										  vertX[i],vertY[i],vertX[(i+1)%n],vertY[(i+1)%n]);
			if(cross)nCross++;
		}else if(Hmap[i]==even&&Hmap[(i+1)%n]==even){
			//如果边的两个端点都在射线上，则此边对于些射线来说退化成为一个点
			//所以穿过次数计为0
		}else{
			//如果边恰有一个端点在射线上，则记为穿过半次
			//(另一个与之共点的边也被记为穿过半次，于是两个合起来就凑成一次了)
			nCross+=0.5;
		}
	}
	//根据Hmap计算极值数
	int nseg=0;//Hmap上!noValue的区段数
	//之所以要记录这个值是因为
	//下面方法只有在nseg!=0时才得到有效结果
	//而当nseg=0时，不用再计算，可直接判断为OUT
	//注：Hmap上全为noValue或者全为!noValue时均认为nseg=0
	int i=0;//遍历Hmap的指针
	while(1){
		//找下一个seg
		//找下一个noValue:!noValue交界
		for(;i<=n-1;i++){
			if(Hmap[i]==noValue&&Hmap[(i+1)%n]!=noValue)break;
		}
		if(i<=n-1){//break出来的，找到了i:i+1即所求
			nseg++;
			//统计!noValue的个数
			int len=0;
			for(int u=i+1;u<=i+1+n-1;u++){
				if(Hmap[u%n]!=noValue)len++;
				else break;
			}//得到len
			//计算区间[(i+1)%n,(i+1+len-1)%n]上的极值个数
			//为了判断区间端点处是极值点还是拐点需要用到区间的外邻居点i和i+1的高度值
			//求Hmap[i]
			if(vertY[i]>y)Hmap[i]=high;
			else if(vertY[i]==y)Hmap[i]=even;
			else Hmap[i]=low;
			//求Hmap[(i+1+len)%n]
			if(vertY[(i+1+len)%n]>y)Hmap[(i+1+len)%n]=high;
			else if(vertY[(i+1+len)%n]==y)Hmap[(i+1+len)%n]=even;
			else Hmap[(i+1+len)%n]=low;
			//边界情况
			if(Hmap[i]==even&&Hmap[(i+1)%n]==even)return false;
			if(Hmap[(i+1+len-1)%n]==even&&Hmap[(i+1+len)%n]==even)return false;
			//中间
			int Hf;
			bool noweven=false;
			for(int j=i+1;j<=i+1+len;j++){//遍历len+1个元素
				if(Hmap[j%n]==even){
					if(noweven==false){//是even上沿
						Hf=Hmap[(j-1)%n];//记下even前一点的高低
						noweven=true;
					}
				}else{
					if(noweven==true){//是even下沿
						if(Hmap[j%n]==Hf)XV++;//是极值
						noweven=false;
					}
				}
			}//本seg上的极值数统计完毕
			//恢复Hmap[i]和Hmap[(i+1+len)%n]
			Hmap[i]=noValue;
			Hmap[(i+1+len)%n]=noValue;
			//更新i
			i=i+1+len;
		}else{//没找到
			break;
		}
	}
	if(nseg==0){//若nseg=0，上面方法根本就是无效的,
		//所以此时得到的XV=0也是无效结果,
		//不用管它，直接返回OUT即可
		return false;
	}
	int nRealCross=nCross+XV;//实际的穿透数
	if(nRealCross%2==1){
		return true;
	}else{
		return false;
	}
}

bool BasicStruct::LineSegOverLineSeg(double x1,double y1,double x2,double y2,double u1,double v1,double u2,double v2)
{//判断段段(x1,y1)-(x2,y2)和线段(u1,v1)-(u2,v2)是否相交
	//判断(u1,v1)和(u2,v2)是否在直线(x1,y1)-(x2,y2)异同侧
	bool u1v1_separate_u2v2=false;
	float a=(v1-y1)*(x2-x1)-(y2-y1)*(u1-x1);
	float b=(v2-y1)*(x2-x1)-(y2-y1)*(u2-x1);
	if(a*b<0)u1v1_separate_u2v2=true;
	//判断(x1,y1)和(x2,y2)是否在直线(u1,v1)-(u2,v2)异同侧
	bool x1y1_separate_x2y2=false;
	a=(y1-v1)*(u2-u1)-(v2-v1)*(x1-u1);
	b=(y2-v1)*(u2-u1)-(v2-v1)*(x2-u1);
	if(a*b<0)x1y1_separate_x2y2=true;
	return x1y1_separate_x2y2*u1v1_separate_u2v2;
}

bool BasicStruct::LineSegOverPolygon(vector<double> vertX, vector<double> vertY, double x1, double y1, double x2,
									 double y2) {
	if(PointInPolygon(vertX,vertY,x1,y1))
	{
		return true;
	}
	if(PointInPolygon(vertX,vertY,x2,y2))
	{
		return true;
	}
	auto n = vertX.size();
	for(int i=0;i<=n-1;i++)
	{
		//判断边(i,i+1)和边(j,j+1)是否相交
		if(LineSegOverLineSeg(vertX[i],vertY[i],vertX[(i+1)%n],vertY[(i+1)%n],
							  x1,y1,x2,y2))
		{
			return true;
		}
	}
	return false;
}
