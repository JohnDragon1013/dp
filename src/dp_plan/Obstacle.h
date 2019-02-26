//
// Created by jydragon on 18-7-23.
//

#ifndef LATTICEPLAN_OBSTACLE_H
#define LATTICEPLAN_OBSTACLE_H

#include "../BasicStruct.h"
#include "../msgHeader/obs2segMsg.h"
struct DynamicObstacle{
    float velocity_;
    RoadPoint Polygon[4];
    float area_;
};
class Obstacle {
public:
    Obstacle()= default;
    //~Obstacle()= default;
    DynamicObstacle GetObstacleInfo(){return m_Obs;}
    //unsigned int GetNumStaticObs(){return num_staticObs;}
    inline vector<unsigned char> GetGridObs(){return m_GridObs;}
    void SetGridObsInfo(const lidar_perception::obs2segMsgPtr& cloud_ptr);
    void SetGridObsInfo();
    //collision with the obstacle
    bool DetectGridObs(Car myCar,RoadPoint &collisionpoint);
    bool DetectObs_Polygon(Car mycar);

private:
    vector<unsigned char> m_GridObs;
    RoadPoint m_currentLocation;
    int grid_xl, grid_xr, grid_yu, grid_yd;
    int GRID_WidthNum,GRID_LengthNum,GRID_Num;
    double grid_widthX,grid_widthY;
protected:
    DynamicObstacle m_Obs;
};


#endif //LATTICEPLAN_OBSTACLE_H
