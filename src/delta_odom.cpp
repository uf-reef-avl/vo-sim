//
// Created by prashant on 9/14/21.
//

#include "../include/delta_odom.h"

#define DBG(msg, ...) { if (verbose_) ROS_WARN_STREAM((msg), ##__VA_ARGS__);}

namespace delta_odom{

    deltaOdom::deltaOdom():
    nh_(),
    nh_private_("~"),
    initialized_(false)
    {
        nh_private_.param<bool>("convert_to_ned", convert_to_ned_, false );
        nh_private_.param<bool>("verbose", verbose_, false );
        nh_private_.param<bool>("add_noise", add_noise_, false );
        nh_private_.param<double>("mocap_noise_std", mocap_noise_std, 0.001);
        nh_private_.param<double>("update_rate", rate, 100);
        nh_private_.param<double>("position_threshold", position_threshold, 0.25);
        nh_private_.param<double>("yaw_threshold", yaw_threshold, 0.1);
        nh_private_.param<bool>("verify_implementation", verify_implementation, false);

        if(verify_implementation)
            integrated_odom_publisher = nh_.advertise<geometry_msgs::PoseStamped>("integrated_odom",1);

        distribution =  std::normal_distribution<>(0,mocap_noise_std);
        rand_numb = rand() % 100;
        std::mt19937 engine(rand_numb);


        pose_stamped_subs_ = nh_.subscribe<geometry_msgs::PoseStamped>("pose_stamped",1 , &deltaOdom::truth_callback, this);
        nav_odom_subs_ = nh_.subscribe<nav_msgs::Odometry>("odom",1 , &deltaOdom::odom_callback, this);
        transform_stamped_subs_ = nh_.subscribe<geometry_msgs::TransformStamped>("transform_stamped",1 , &deltaOdom::transform_callback, this);

        true_odom_publisher = nh_.advertise<geometry_msgs::PoseStamped>("true_odom",1);
        noisy_odom_publisher = nh_.advertise<geometry_msgs::PoseStamped>("noisy_odom",1);
    }

    deltaOdom::~deltaOdom() {

    }

    void deltaOdom::truth_callback(const geometry_msgs::PoseStampedConstPtr &msg) {
        Eigen::Affine3d pose_msg;
        tf2::fromMsg(msg->pose, pose_msg);

        if(verify_implementation){
            if(!initialized_)
                current_pose = pose_msg;
        }

        process_msg(pose_msg, msg->header);
    }

    void deltaOdom::transform_callback(const geometry_msgs::TransformStampedConstPtr &msg) {

        Eigen::Affine3d pose_msg;
        pose_msg = tf2::transformToEigen(*msg);
        process_msg(pose_msg, msg->header);
    }

    void deltaOdom::odom_callback(const nav_msgs::OdometryConstPtr &msg) {
        Eigen::Affine3d pose_msg;
        tf2::fromMsg(msg->pose.pose, pose_msg);
        process_msg(pose_msg, msg->header);
    }

    void deltaOdom::process_msg(Eigen::Affine3d &optitrack_to_current_pose, std_msgs::Header header) {

        if(!initialized_){
            previous_time = header.stamp.toSec();
            optitrack_to_preivous_pose = optitrack_to_current_pose;
            initialized_ = true;
            return;
        }

        Eigen::Affine3d delta_pose_odom;

        delta_pose_odom = optitrack_to_preivous_pose.inverse() * optitrack_to_current_pose;

        double delta_position = delta_pose_odom.translation().norm();
        double delta_yaw;
        reef_msgs::get_yaw(delta_pose_odom.linear().transpose(), delta_yaw);
        DT = header.stamp.toSec() - previous_time;

        if(delta_position >= position_threshold || delta_yaw >= yaw_threshold || DT >= 1/rate){
            DBG("Reset Odom");

            previous_time = header.stamp.toSec();
            optitrack_to_preivous_pose = optitrack_to_current_pose;

            geometry_msgs::PoseStamped true_delta;
            true_delta.header = header;
            true_delta.pose = tf2::toMsg(delta_pose_odom);
            true_delta.pose.position.z = 0;
            true_odom_publisher.publish(true_delta);

//            Eigen::Matrix<double, 3, 1> ea;
//            ea << 0,0,delta_yaw;
//            true_delta.pose.orientation = reef_msgs::fromEulerAngleToQuaternion<Eigen::Matrix<double, 3, 1>, geometry_msgs::Quaternion> (ea, "321");
            if(verify_implementation){
                Eigen::Affine3d temp;
                temp = current_pose * delta_pose_odom;
                current_pose = temp;
                geometry_msgs::PoseStamped verify_msg;
                verify_msg.pose = tf2::toMsg(current_pose);
                integrated_odom_publisher.publish(verify_msg);
            }

            if(add_noise_){
                true_delta.pose.position.x += distribution(engine);
                true_delta.pose.position.y += distribution(engine);
                noisy_odom_publisher.publish(true_delta);
            }
        }

    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "delta_odom_node");
    delta_odom::deltaOdom object;
    ros::spin();
    return 0;
}