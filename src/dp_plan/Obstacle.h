//
// Created by jydragon on 18-7-23.
//

#ifndef LATTICEPLAN_OBSTACLE_H
#define LATTICEPLAN_OBSTACLE_H

#include "../BasicStruct.h"
#include "../msgHeader/Grid.h"
class Obstacle {
public:
    Obstacle()= default;
    ~Obstacle()= default;
    //vector <Obstacle> GetObstacleInfo(){return m_Obs;}
    //unsigned int GetNumStaticObs(){return num_staticObs;}
    inline vector<unsigned char> GetGridObs(){return m_GridObs;}
    void SetGridObsInfo(const iau_ros_msgs::GridPtr& cloud_ptr);
    void SetGridObsInfo();
    //collision with the obstacle
    bool DetectGridObs(Car myCar,RoadPoint &collisionpoint);

private:
    vector<unsigned char> m_GridObs;
    RoadPoint m_currentLocation;
    int grid_xl, grid_xr, grid_yu, grid_yd;
    int GRID_WidthNum,GRID_LengthNum,GRID_Num;
    double grid_widthX,grid_widthY;
};


#endif //LATTICEPLAN_OBSTACLE_H
