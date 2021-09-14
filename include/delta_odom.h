//
// Created by prashant on 9/14/21.
//

#ifndef DELTA_ODOM_H
#define DELTA_ODOM_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>
#include <reef_msgs/matrix_operation.h>
#include <reef_msgs/AngleRepresentationInterface.h>
#include <reef_msgs/dynamics.h>
#include <tf2_eigen/tf2_eigen.h>
#include <random>

namespace delta_odom{
    class deltaOdom{
    private:
        ros::NodeHandle nh_private_;
        ros::NodeHandle nh_;
        ros::Publisher true_odom_publisher;
        ros::Publisher noisy_odom_publisher;

        ros::Subscriber pose_stamped_subs_;
        ros::Subscriber nav_odom_subs_;
        ros::Subscriber transform_stamped_subs_;

        void truth_callback(const geometry_msgs::PoseStampedConstPtr &msg);
        void odom_callback(const nav_msgs::OdometryConstPtr &msg);
        void transform_callback(const geometry_msgs::TransformStampedConstPtr &msg);
        void process_msg(Eigen::Affine3d& pose_msg, std_msgs::Header header);

        bool convert_to_ned_;
        bool initialized_;
        bool verbose_;
        bool add_noise_;

        double DT;
        double yaw;
        double rate;
        double previous_time;
        double position_threshold;
        double yaw_threshold;

        Eigen::Affine3d optitrack_to_preivous_pose;
        Eigen::Vector3d noise_vector;

        int rand_numb;
        std::mt19937 engine;
        std::normal_distribution<double> distribution;
        double mocap_noise_std;


    public:
        deltaOdom();
        ~deltaOdom();

    };
}
#endif //DELTA_ODOM_H
