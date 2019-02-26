# latticePlan

DP planing   
## before run
You need to find a global reference path which should include (lon , lat , theta).The subsequent version will need (k, velocity) and some other parameters.    
The program will check the global path.   
Notice: the file name is a absolute path,if you want to change file ,you need to rewrite th file path in latticeplan.cpp.   

##rosbag  
 
There is a global path in the program(src/testmap.txt),the velodyne_points data can be found in NAS 20180616_yby.The path is corresponding to a 6.1G bag.

## run lidar_obstacle

roslaunch velodyne_pointscloud momenta_64e

## open rviz
rviz open configure <${package_dir}/rviz/plan.rviz>
