#ifndef _FRONT_ODOMETRY_HPP_
#define _FRONT_ODOMETRY_HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/registration/ndt.h>

#include <time.h>           
#include <eigen3/Eigen/Dense>
#include <glog/logging.h>
namespace front_odom
{
    typedef pcl::PointXYZ PointType;

    class front_odometry
    {
        public:
            front_odometry();
        private:
        
            ros::NodeHandle nh_;
            ros::Subscriber points_sub_;      

            pcl::PointCloud<PointType> map_;
            pcl::VoxelGrid<PointType> voxel_grid_filter_;
            pcl::NormalDistributionsTransform<PointType, PointType> ndt;

            int max_iter_ ;         
            double ndt_res_ ;       
            double step_size_ ;    
            double trans_eps_ ;   

            double voxel_leaf_size_; 

            // 发布
            ros::Publisher ndt_map_pub_;
            ros::Publisher pubLaserOdometry_,pubLaserPath_;     // Odometry发布、Path发布
            ros::Publisher pubCurrentCloud_;                    // 发布当前帧点云
            tf::TransformBroadcaster br_;
            double min_add_scan_shift_;

            Eigen::Matrix4d T_w_curr;             // 当前帧pose
            Eigen::Matrix4d guess_pose;           // guess_pose
            Eigen::Matrix4d previous_pose;        // 上一个关键帧的pose

            bool is_first_frame;                  // 是否是第一帧数据

            void laserCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in);
        };              
}               


#endif

