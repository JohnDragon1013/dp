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
1 直线路径规划 无障碍物情况下  
这种就是对比两种算法的规划速率，平滑程度，和侧向加速度值等。（测试较为方便）  
2 避障情况  
这种是要回到原车道，判断路径的连续程度，方向盘转角的连续性。（可以仿真）  
3 超车换道
切换到旁边车道，一般向左变道。这里需要测量前车距离和速度！！！（仿真或实测）  
这里记录下前车的距离，然后估计前车速度。  
这里还有一个问题，需要将车道换回去。判断车后是否有障碍物，安全距离切换回车道。
