#include <ros/ros.h>
#include <iostream>
#include <queue>
#include <string>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <Eigen/Eigen>
#include "front_odometry.hpp"

namespace front_odom
{
    front_odometry::front_odometry()
    {
        points_sub_ = nh_.subscribe("/kitti/velo/pointcloud", 100000, &front_odometry::laserCallback,this);

        ndt_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
        pubLaserOdometry_ = nh_.advertise<nav_msgs::Odometry>("/laser_odom_to_map",1000);
        pubLaserPath_ = nh_.advertise<nav_msgs::Path>("/laser_odom_path", 1000);
        pubCurrentCloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/current_cloud", 1000);

        nh_.param("max_iter", max_iter_, 30);
        nh_.param("step_size", step_size_, 0.1);
        nh_.param("ndt_res", ndt_res_, 5.0);
        nh_.param("trans_eps", trans_eps_, 0.01);
        nh_.param("voxel_leaf_size", voxel_leaf_size_, 2.0);
        nh_.param("min_add_scan_shift",  min_add_scan_shift_,1.0);

        T_w_curr = Eigen::Matrix4d::Identity();              // 当前帧pose
        guess_pose = Eigen::Matrix4d::Identity();            // guess_pose
        previous_pose = Eigen::Matrix4d::Identity();         // 上一个关键帧的pose

        is_first_frame = true;//是否第一帧

        // voxel ndt的参数
        voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);  
        ndt.setTransformationEpsilon(trans_eps_);
        ndt.setStepSize(step_size_);
        ndt.setResolution(ndt_res_);
        ndt.setMaximumIterations(max_iter_);

        std::cout << "ndt_res: " << ndt_res_ << std::endl;
        std::cout << "step_size: " << step_size_ << std::endl;
        std::cout << "trans_epsilon: " << trans_eps_ << std::endl;
        std::cout << "max_iter: " << max_iter_ << std::endl;
        std::cout << "voxel_leaf_size: " << voxel_leaf_size_ << std::endl;
        std::cout << "min_add_scan_shift: " << min_add_scan_shift_ << std::endl;

    }

    void front_odometry::laserCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
    {
        // 转换到pcl格式
        pcl::PointCloud<PointType> tmp,scan;
        pcl::fromROSMsg(*cloud_in,scan);

        if(is_first_frame)          // 第一帧
        {
            is_first_frame = false;
            
            map_ += scan;
            ndt.setInputTarget(map_.makeShared());
        }

        // 点云降采样
        pcl::PointCloud<PointType>::Ptr filtered_scan_ptr(new pcl::PointCloud<PointType>);
        voxel_grid_filter_.setInputCloud(scan.makeShared());
        voxel_grid_filter_.filter(*filtered_scan_ptr);

        // filtered_scan_ptr和map_配准
        pcl::PointCloud<PointType>::Ptr output_cloud_ptr(new pcl::PointCloud<PointType>);
        ndt.setInputSource(filtered_scan_ptr);
        ndt.align(*output_cloud_ptr,guess_pose.cast<float>());
        T_w_curr = ndt.getFinalTransformation().cast<double>();

        // 更新guess_pose
        guess_pose = T_w_curr;

        // 矩阵转为q和t
        Eigen::Matrix3d R = T_w_curr.block<3,3>(0,0);
        static Eigen::Quaterniond q_w_curr;
        static Eigen::Vector3d t_w_curr;
        q_w_curr = Eigen::Quaterniond(R);
        t_w_curr = T_w_curr.block<3,1>(0,3);

        //计算移动距离
        double shift = sqrt(pow(T_w_curr(0,3) - previous_pose(0,3), 2.0) + pow(T_w_curr(1,3) - previous_pose(1,3), 2.0));   
        if(shift > min_add_scan_shift_) 
        {
            // 更新关键帧
            previous_pose = T_w_curr;

            // 更新map
            pcl::PointCloud<PointType>::Ptr transfromed_cloud_ptr(new pcl::PointCloud<PointType>);
            pcl::transformPointCloud(scan,*transfromed_cloud_ptr,T_w_curr.cast<float>());
            map_ += *transfromed_cloud_ptr;

            // 更新target
            ndt.setInputTarget(map_.makeShared());

            // 发布地图
            sensor_msgs::PointCloud2 map_ros;
            pcl::toROSMsg(map_,map_ros);
            map_ros.header.frame_id = "map";
            ndt_map_pub_.publish(map_ros);
        }

        // 发布当前帧点云
        sensor_msgs::PointCloud2 current_cloud_ros;
        pcl::toROSMsg(tmp,current_cloud_ros);
        current_cloud_ros.header.frame_id = "map_child";
        pubCurrentCloud_.publish(current_cloud_ros);

        //输出位姿和path
        nav_msgs::Odometry laserOdometry;
        laserOdometry.header.frame_id = "map";
        laserOdometry.child_frame_id = "map_child";         // 当前帧
        laserOdometry.header.stamp = cloud_in->header.stamp;
        laserOdometry.pose.pose.orientation.x = q_w_curr.x();           // 当前帧的pose
        laserOdometry.pose.pose.orientation.y = q_w_curr.y();
        laserOdometry.pose.pose.orientation.z = q_w_curr.z();
        laserOdometry.pose.pose.orientation.w = q_w_curr.w();
        laserOdometry.pose.pose.position.x = t_w_curr.x();
        laserOdometry.pose.pose.position.y = t_w_curr.y();
        laserOdometry.pose.pose.position.z = t_w_curr.z();
        pubLaserOdometry_.publish(laserOdometry);

        // 输出path
        geometry_msgs::PoseStamped laserPose;
        nav_msgs::Path laserPath;
        laserPose.header = laserOdometry.header;
        laserPose.pose = laserOdometry.pose.pose;
        laserPath.header.stamp = laserOdometry.header.stamp;
        laserPath.poses.push_back(laserPose);
        laserPath.header.frame_id = "map";
        pubLaserPath_.publish(laserPath); 

        tf::Transform transform;
        tf::Quaternion q;
        transform.setOrigin(tf::Vector3(t_w_curr.x(),t_w_curr.y(),t_w_curr.z()));
        q.setW(q_w_curr.w());
        q.setX(q_w_curr.x());
        q.setY(q_w_curr.y());
        q.setZ(q_w_curr.z());
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, cloud_in->header.stamp, "map", "map_child"));        // 发布world到当前帧的tf变换
    }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "front_odometry");
  front_odom::front_odometry ndt_front_odom;

  ros::spin();

  return 0;
};