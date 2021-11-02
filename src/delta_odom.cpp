//
// Created by prashant on 9/14/21.
//

#include "../include/delta_odom.h"

#define DBG(msg, ...) { if (verbose_) ROS_WARN_STREAM((msg), ##__VA_ARGS__);}

namespace delta_odom{

    deltaOdom::deltaOdom():
    nh_(),
    nh_private_("~"),
    initialized_(false),
    keyframe_now(false)
    {
        nh_private_.param<bool>("convert_to_ned", convert_to_ned_, false );
        nh_private_.param<bool>("verbose", verbose_, false );
        nh_private_.param<bool>("add_noise", add_noise_, false );
        nh_private_.param<double>("mocap_noise_std", mocap_noise_std, 0.001);
        nh_private_.param<double>("update_rate", rate, 20);
        nh_private_.param<double>("position_threshold", position_threshold, 0.25);
        nh_private_.param<double>("yaw_threshold", yaw_threshold, 0.1);
        nh_private_.param<bool>("verify_implementation", verify_implementation, false);

        reef_msgs::loadTransform("body_to_camera",body_to_camera);

        if(verify_implementation)
            integrated_odom_publisher = nh_.advertise<geometry_msgs::PoseStamped>("integrated_odom",1);

        distribution =  std::normal_distribution<>(0,mocap_noise_std);
        rand_numb = rand() % 100;
        std::mt19937 engine(rand_numb);

        //std::string initialize_node_topic;
        //nh_private_.param<std::string>("initialize_node_topic", initialize_node_topic);
        //std_msgs::Empty::ConstPtr initialize_node = ros::topic::waitForMessage<std_msgs::Empty>(initialize_node_topic, nh_);

        pose_stamped_subs_ = nh_.subscribe<geometry_msgs::PoseStamped>("pose_stamped",1 , &deltaOdom::truth_callback, this);
        nav_odom_subs_ = nh_.subscribe<nav_msgs::Odometry>("odom",1 , &deltaOdom::odom_callback, this);
        transform_stamped_subs_ = nh_.subscribe<geometry_msgs::TransformStamped>("transform_stamped",1 , &deltaOdom::transform_callback, this);
        keyframe_subs_ = nh_.subscribe<std_msgs::Empty>("keyframe_now",1, &deltaOdom::keyframeCallback, this);

        true_odom_publisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("true_odom",1);
        noisy_odom_publisher = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("noisy_odom",1);
        altimeter_publisher_ = nh_.advertise<sensor_msgs::Range>("altimeter", 1);
    }

    deltaOdom::~deltaOdom() {

    }

    void deltaOdom::keyframeCallback(const std_msgs::EmptyConstPtr &msg) {
        keyframe_now = true;
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
		

        Eigen::Affine3d delta_pose_odom_optitrack_frame;
        Eigen::Affine3d delta_pose_odom_camera_frame;

        sensor_msgs::Range alt_msg;
        alt_msg.header = header;
        alt_msg.range = optitrack_to_current_pose.translation().z();

        delta_pose_odom_optitrack_frame= optitrack_to_preivous_pose.inverse() * optitrack_to_current_pose;
        delta_pose_odom_camera_frame = body_to_camera.inverse() * delta_pose_odom_optitrack_frame * body_to_camera;
        DT = header.stamp.toSec() - previous_time;

        if(keyframe_now){
            DBG("Reset Odom");

            optitrack_to_preivous_pose = optitrack_to_current_pose;
            keyframe_now = false;

            if(verify_implementation){
                Eigen::Affine3d temp;
                temp = current_pose * delta_pose_odom_optitrack_frame;
                current_pose = temp;
                geometry_msgs::PoseStamped verify_msg;
                verify_msg.pose = tf2::toMsg(current_pose);
                integrated_odom_publisher.publish(verify_msg);
            }
        }
        // Publish the delta_odom value w.r.t keyframe
        if(DT>1/rate) {
            geometry_msgs::PoseWithCovarianceStamped true_delta;
            true_delta.header = header;
            true_delta.pose.pose = tf2::toMsg(delta_pose_odom_camera_frame);
            //TODO: Add covariance
            true_odom_publisher.publish(true_delta);

            if (add_noise_) {
                true_delta.pose.pose.position.x += distribution(engine);
                true_delta.pose.pose.position.y += distribution(engine);
                alt_msg.range += distribution(engine);
                noisy_odom_publisher.publish(true_delta);
            }
            altimeter_publisher_.publish(alt_msg);
            previous_time = header.stamp.toSec();
        }
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "delta_odom_node");
    delta_odom::deltaOdom object;
    ros::spin();
    return 0;
}