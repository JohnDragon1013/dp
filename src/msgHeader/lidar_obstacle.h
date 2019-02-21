//
// Created by hdmap on 18-6-12.
//

#ifndef LIDAR_PERCEPTION_LIDAR_DETECTION_H
#define LIDAR_PERCEPTION_LIDAR_DETECTION_H
#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <unistd.h>
#include <chrono>
//yaml
#include <yaml-cpp/yaml.h>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//ros
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
//#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <cv_bridge/cv_bridge.h>
//msg
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "lidar_perception/obs2segMsg.h"
//project
#include "groundDetection/generateGrid2d.h"
#include "scanRegister/multiScanRegister.h"
#include <utils/marker_array_pub.hpp>

typedef std::chrono::high_resolution_clock chtime;
typedef std::chrono::milliseconds chms;

static int frame = 0;
static float  total_time = 0;
namespace lidar_perception
{
    class LidarObstacle
    {
    public:
        LidarObstacle():_cloud_update(new pcl::PointCloud<PointXYZI>),
                        _time_update(ros::Time::now()),
                        _flag_update_msg(false)
        {
            initTopicMap();
        }
        ~LidarObstacle(){}



        //function

    private:


        void process()
        {

            frame++;
            //--------------obstacle grid-----------------//
            chtime::time_point time10 = chtime::now();
//            double time10 = pcl::getTime();
            _gridGenerator.setInputCloud(_cloud_register);
            _gridGenerator.setIndex2Origin(_index_2origin);
            _gridGenerator.generateGrid();
            _gridGenerator.gridObstacleDetect();


            //----------ros publish obs2segment--------//
            std::vector<unsigned> index_obstacleCloud = _gridGenerator.getObstacleIndex();
            pcl::PointCloud<PointType>::Ptr cloud_obstacle = getCloudwithIndex(_cloud_register, index_obstacleCloud);
            chtime::time_point time11 = chtime::now();
            chms timeInterval1 = std::chrono::duration_cast<chms>(time11 - time10);

            std::cout<<"## time_obs "<<timeInterval1.count() <<" ms"<<std::endl;
            total_time += timeInterval1.count();

            std::cout<<"## frame: Average time_obs "<<frame << " " << total_time / frame<<" ms"<<std::endl;
            //发送部分
            publishCloud<PointType>(cloud_obstacle, _pub_cloud_obstacle);

            //temp....
            std::vector<unsigned> index_GridRemovePoint = _gridGenerator.getGridRemovePointIndex();
            pcl::PointCloud<PointType>::Ptr cloud_GridRemovePoint = getCloudwithIndex(_cloud_register, index_GridRemovePoint);
            publishCloud<PointType>(cloud_GridRemovePoint, _pub_cloud_GridRemovePoint);

            std::vector<unsigned> index_float = _gridGenerator.getfloatIndex();
            pcl::PointCloud<PointType>::Ptr cloud_float = getCloudwithIndex(_cloud_register, index_float);
            publishCloud<PointType>(cloud_float, _pub_cloud_float);

            chtime::time_point time12 = chtime::now();
            publish_obs2seg();
            chtime::time_point time13 = chtime::now();

            chms timeInterval2 = std::chrono::duration_cast<chms>(time13 - time12);
            chms TotaltimeInterval = std::chrono::duration_cast<chms>(time13 - time10);
            std::cout<<"## time------------------------TOTAL: "<<TotaltimeInterval.count()<<" ms"<<" "<<timeInterval2.count()<<" ms"<<std::endl;
        }



        void publish_obs2seg()
        {
            //if(_pub_2segment.getNumSubscribers() > 0)
            {
//                chtime::time_point time0 = chtime::now();

                lidar_perception::obs2segMsg rosMsg_2segment;
                rosMsg_2segment.header.stamp = _time_register;
                pcl::toROSMsg(*_cloud_register, rosMsg_2segment.cloud);
                rosMsg_2segment.grid_rows = _gridGenerator._grid_flagobs.rows();
                rosMsg_2segment.grid_cols = _gridGenerator._grid_flagobs.cols();

                //grid_flag
                for(unsigned i = 0 ; i < _gridGenerator._grid_flagobs.rows(); i++)
                {
                    for(unsigned j = 0 ; j < _gridGenerator._grid_flagobs.cols(); j++)
                    {
                        if(_gridGenerator._grid_flagobs(i,j))
                            rosMsg_2segment.grid_flag.push_back(1);
                        else
                            rosMsg_2segment.grid_flag.push_back(0);
                    }
                }

 //               chtime::time_point time1 = chtime::now();
                //cloud_flag
                rosMsg_2segment.cloud_flag = _gridGenerator._labelImg;

                chtime::time_point time2 = chtime::now();
                //cloud_gridID
                for(unsigned i = 0 ; i < _gridGenerator._cloud_gridID.size() ; i++)
                {
                    lidar_perception::cloud_index cloud_index_temp;
                    cloud_index_temp.row =  _gridGenerator._cloud_gridID[i].first;
                    cloud_index_temp.col =  _gridGenerator._cloud_gridID[i].second;
                    rosMsg_2segment.cloud_gridID.push_back(cloud_index_temp);
                }

                chtime::time_point time3 = chtime::now();
                //grid_cloudID
                int count = 0 ;
                rosMsg_2segment.grid_cloudID.resize(rosMsg_2segment.grid_cols * rosMsg_2segment.grid_rows);
                for(unsigned i = 0 ; i < _gridGenerator._grid_cloudID.rows() ; i++)
                {
                    for(unsigned j = 0 ; j < _gridGenerator._grid_cloudID.cols() ; j++)
                    {
                        lidar_perception::grid_index grid_index_temp;
                        grid_index_temp.index_cloudID.resize(_gridGenerator._grid_cloudID(i,j).size());
                        for(unsigned p = 0 ; p < _gridGenerator._grid_cloudID(i,j).size() ; p++)
                            grid_index_temp.index_cloudID[p] = (_gridGenerator._grid_cloudID(i,j)[p]);
                        grid_index_temp.row = i;
                        grid_index_temp.col = j;
                        rosMsg_2segment.grid_cloudID[count] = (grid_index_temp);

                        count++;
                    }
                }
                chtime::time_point time4 = chtime::now();


                _pub_2segment.publish(rosMsg_2segment);
//                chtime::time_point time5 = chtime::now();

                std::cout<<"time3: "<<std::chrono::duration_cast<chms>(time4 - time3).count()<<std::endl;
            }
        }


        void callbackRos(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
        {
            /**  **/

            pcl::PointCloud<PointXYZI>::Ptr cloud_pcl(new pcl::PointCloud<PointXYZI>);
            pcl::fromROSMsg(*cloud_ptr, *cloud_pcl);

            std::cout<<std::setprecision(12)<<"[lidar_detection] -cloud num = "<<cloud_pcl->points.size()<<" -time = "
                     <<cloud_ptr->header.stamp.toSec()<<std::endl;

            {
                boost::mutex::scoped_lock lock (_mutex_update);
                _cloud_update = cloud_pcl;
                _time_update = cloud_ptr->header.stamp;
                _flag_update_msg = true;
                ROS_INFO("rostopic update cloud \n");

                _mutex_update.unlock();
            }


        }

    public:
        void callbackProcess()
        {
            while(ros::ok())
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));
                {
                    boost::mutex::scoped_lock lock(_mutex_update);

                    if (_cloud_update->points.size() == 0 || fabs(_time_register.toSec() - _time_update.toSec()) < 0.01)
                        continue;
                    //--------------register cloud--------------//
                    //data update
                    _cloud_register = _multiScanRegister.reorderCloud(_cloud_update);
                    _index_2origin = _multiScanRegister.getIndex2Orign();
                    _time_register = _time_update;

                    ROS_INFO("lidar_obstacle process cloud \n");
                }
                process();
            }

        }

    public:

        void setParam(LidarType lidarType, std::string paramFolder)
        {
            _lidar_type = lidarType;
            _param_dir = paramFolder;
        }

        void setParam_fromNode(ros::NodeHandle &privateNode)
        {
            std::string sparam;
            if(privateNode.getParam("param_folder", sparam))
            {
                _param_dir = sparam;
                ROS_INFO("Set param_folder: %s", sparam.c_str());
            }

            if(privateNode.getParam("lidar_type", sparam))
            {
                if(sparam.compare("Velo_HDL_64") == 0)
                {
                    _lidar_type = LidarType::Velo_HDL_64;
                }
                else if(sparam.compare("Velo_HDL_32") == 0)
                {
                    _lidar_type = LidarType::Velo_HDL_32;
                }
                else if(sparam.compare( "Velo_VLP_32") == 0)
                {
                    _lidar_type = LidarType::Velo_VLP_32;
                }
                else if(sparam.compare( "Velo_VLP_16") == 0)
                {
                    _lidar_type = LidarType::Velo_VLP_16;
                }
                else if(sparam.compare( "Pandar_40") == 0)
                {
                    _lidar_type = LidarType::Pandar_40;
                }
                else{
                    _lidar_type = LidarType::Velo_HDL_64;
                    ROS_WARN("No match lidar type: %s", sparam.c_str());
                }

                ROS_INFO("Set lidar_type: %s", sparam.c_str());
            }
        }


        bool setRosCommunication(ros::NodeHandle &nh)
        {
            //communicate with segment
            _pub_2segment = nh.advertise<lidar_perception::obs2segMsg>(_ros_topic["obs2segment"], 2);

            //debug
            _pub_cloud_obstacle = nh.advertise<sensor_msgs::PointCloud2>(_ros_topic["cloud_obstacle"], 2);

            _pub_cloud_GridRemovePoint = nh.advertise<sensor_msgs::PointCloud2>(_ros_topic["cloud_gridmovepoint"], 2);

            _pub_cloud_float = nh.advertise<sensor_msgs::PointCloud2>(_ros_topic["cloud_float"], 2);
            return true;

        }
        void chatterCallback(const lidar_perception::obs2segMsg &msg)
        {
            ROS_INFO("I heard odom:[%s]",msg.grid_rows);
            std::cout<<" chatterCallback run"<<std::endl;
        }

        void startRosCommunicateMode(ros::NodeHandle &nh)
        {
            /** set subscribe ros topic **/

           // ROS_INFO("POSE Topic: %s", _ros_topic["lidar_slam"].c_str());
           // ROS_INFO("Cloud Topic: %s", _ros_pointTopic.c_str());

            //_sub_pose = nh.subscribe(_ros_topic["lidar_slam"], 5,  &PoseServer::callbackPoseSub, &_poseServer);
            _sub_cloud = nh.subscribe(_ros_pointTopic, 2,  &LidarObstacle::chatterCallback, this);

            //boost::function0<void> f = boost::bind(&LidarObstacle::callbackProcess,this);
            //boost::thread thread_viewer(f);
            //thread_viewer.detach();

        }


        void init()
        {
            /** set lidar_type, param_folder, read in map **/

            std::string yaml_path = _param_dir + "/topic.yaml";
            if(access(yaml_path.c_str(),0) == -1) {
                std::stringstream ss;
                ss << "mkdir -p " <<_param_dir << "/param";
                std::cout<<ss.str().c_str()<<std::endl;
                int back = system(ss.str().c_str());
                writeYAML(_param_dir + "/param/topic.yaml");
            }
            else
                readYAML(yaml_path);

            //_mapManger.setMapRootDir(_param_dir + "/roadMap/");


            //lidar configure
            switch (_lidar_type) {
                case LidarType::Velo_HDL_64:
                    _rviz_frame_id = "velodyne";
                    _ros_pointTopic = "/velodyne_points";
                    _lidar_height = 1.59;
                    _lidar_configure = getConfigure(_param_dir + "/lidar/64db.yaml", _lidar_type);
                    break;
                case LidarType::Pandar_40:
                    _rviz_frame_id = "pandar";
                    _ros_pointTopic = "/pandar_points";
                    _lidar_height = 1.76;
                    _lidar_configure = getConfigure(_param_dir + "/lidar/40.csv", _lidar_type);
                    break;
                case LidarType ::Velo_VLP_32:
                    _rviz_frame_id = "velodyne";
                    _ros_pointTopic = "/velodyne_points";
                    _lidar_height = 1.75;
                    _lidar_configure = getConfigure(_param_dir + "/lidar/32c.yaml", _lidar_type);
                    break;
                default:
                    ;
            }

            //trans param //waiting

            /** init process class **/

            _multiScanRegister.setLidarConfigure(_lidar_configure);
            _multiScanRegister.setOrganizedCloud();

            _gridGenerator.setGridParam(300,300,500,500,0.2);
//            _gridGenerator.setGridParam(240,240,400,400,0.25);
            _gridGenerator.setDoubleCheck();

        }



    private:

        std::map<std::string,std::string> _ros_topic;
        void initTopicMap()
        {
            /** init ros topic, default setting **/

            _ros_topic.insert(std::pair<std::string, std::string>("ground","/lidar_perception/ground"));
            _ros_topic.insert(std::pair<std::string, std::string>("roadRoi","/lidar_perception/roadRoi"));
            _ros_topic.insert(std::pair<std::string, std::string>("segment","/lidar_perception/segment"));
            _ros_topic.insert(std::pair<std::string, std::string>("cluster_bbox","/lidar_perception/cluster_bbox"));
            _ros_topic.insert(std::pair<std::string, std::string>("cloud_feature","/lidar_perception/cloud_feature"));
            _ros_topic.insert(std::pair<std::string, std::string>("cloud_cluster","/lidar_perception/cloud_cluster"));
            _ros_topic.insert(std::pair<std::string, std::string>("cloud_merge","/lidar_perception/cloud_merge"));
            _ros_topic.insert(std::pair<std::string, std::string>("cloud_obstacle","/lidar_perception/cloud_obstacle"));
            //.........
            _ros_topic.insert(std::pair<std::string, std::string>("cloud_gridmovepoint","/lidar_perception/cloud_gridmovepoint"));
            _ros_topic.insert(std::pair<std::string, std::string>("cloud_float","/lidar_perception/cloud_float"));

            _ros_topic.insert(std::pair<std::string, std::string>("lidar_slam","/lidar_to_map"));
            _ros_topic.insert(std::pair<std::string, std::string>("obs2segment","/lidar_obstacle/obs2segment"));

        }

        void readYAML(std::string path)
        {
            /** read local topic settings from yaml **/

            YAML::Node node = YAML::LoadFile(path.c_str());

            if(node["ground"])
                _ros_topic["ground"] = node["ground"].as<std::string>();
            if(node["roadRoi"])
                _ros_topic["roadRoi"] = node["roadRoi"].as<std::string>();
            if(node["segment"])
                _ros_topic["segment"] = node["segment"].as<std::string>();
            if(node["cluster_bbox"])
                _ros_topic["cluster_bbox"] = node["cluster_bbox"].as<std::string>();
            if(node["cloud_feature"])
                _ros_topic["cloud_feature"] = node["cloud_feature"].as<std::string>();
            if(node["cloud_cluster"])
                _ros_topic["cloud_cluster"] = node["cloud_cluster"].as<std::string>();
            if(node["cloud_merge"])
                _ros_topic["cloud_merge"] = node["cloud_merge"].as<std::string>();
            if(node["lidar_slam"])
                _ros_topic["lidar_slam"] = node["lidar_slam"].as<std::string>();
        }

        void writeYAML(std::string path)
        {
            /** write current map into yaml **/

            YAML::Node node;
            for(std::map<std::string,std::string>::iterator it = _ros_topic.begin() ; it != _ros_topic.end() ; it++)
            {
                node[it->first] = it->second;
            }
            std::string content = YAML::Dump(node);
            std::fstream file(path.c_str(), std::ios::out);
            file<<content;
            file.close();
        }


        //variable
    public:

        //class member
    private:
        MultiScanRegister _multiScanRegister;
        //process
        CGridGenerator _gridGenerator;


        //label
    private:
        pcl::PointCloud<PointType>::Ptr _cloud_register;
        std::vector<unsigned> _index_2origin;

        //ros publisher ///_pub
    private:
        //necessary
        ros::Publisher _pub_2segment;
        ros::Subscriber _sub_cloud;
        //debug
        ros::Publisher _pub_cloud_obstacle;
        ros::Publisher _pub_cloud_GridRemovePoint;
        ros::Publisher _pub_cloud_float;
        //frame data
        bool _flag_update_msg;
        pcl::PointCloud<PointXYZI>::Ptr _cloud_update;
        ros::Time _time_update, _time_register;
        boost::mutex _mutex_update;


        //ros publish cloud function
        template <typename PointT>
        void publishCloud(typename pcl::PointCloud< PointT >::Ptr &cloud , ros::Publisher &pub)
        {
            if(pub.getNumSubscribers()==0)
                return;

            sensor_msgs::PointCloud2 rosMsg_cloud;
            pcl::toROSMsg(*cloud, rosMsg_cloud);
            rosMsg_cloud.header.frame_id = _rviz_frame_id;
            pub.publish(rosMsg_cloud);
            return;
        }

        //configure  ///_ros,_rviz, _lidar
    private:
        std::string _param_dir;
        LidarType _lidar_type;
        float _lidar_height;
        LidarConfigure _lidar_configure;
        std::string _rviz_frame_id, _ros_pointTopic;

    };
}

#endif //LIDAR_PERCEPTION_LIDAR_DETECTION_H
