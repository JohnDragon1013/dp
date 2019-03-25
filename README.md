# DP Plan
DP planing without speed plan(master)   
## before run
You need to find a global reference path which should include (lon , lat , theta).The subsequent version will need (k, velocity) and some other parameters.
The program will check the global path.   
#### Notice: 
the file name is a absolute path,if you want to change file ,you need to rewrite th file path in latticeplan.cpp.
There is a global path in the program(src/testmap.txt).
## run lidar_obstacle

receive velodyne point cloud from lidar.

## run map tools

receive multiple lane data from map tools.

## open rviz
rviz open configure <${package_dir}/rviz/tracking.rviz>

=======================================
# 毕业论文
##实验场景   
这个分支是为了仿真实验建立的，具体情况如下

这里可能需要修改，做特殊场景的规划比较 和原来的RRT  仿真实验即可   
1 直道 （无避障 避障）
2 狭窄路口 （常熟土路入口，或者高速入口）
3 高速匝道 （弯道 c型）
4 乡间小路 （弯道 S型）
5 多车道 （换道）   
直接记录topic吧，全部topic。   
需要记录的元素：车速/前车距离/实时位置/障碍物grid

