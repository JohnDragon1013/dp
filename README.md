# DP Plan

DP planing without speed plan(master)   
## before run
You need to find a global reference path which should include (lon , lat , theta).The subsequent version will need (k, velocity) and some other parameters.
The program will check the global path.   
Notice: the file name is a absolute path,if you want to change file ,you need to rewrite th file path in latticeplan.cpp.
There is a global path in the program(src/testmap.txt).

## run lidar_obstacle

receive velodyne point cloud from lidar.


## run map tools

receive multiple lane data from map tools.

## open rviz
rviz open configure <${package_dir}/rviz/tracking.rviz>

=======================================
## 毕业论文
