#include <Eigen/Dense>
#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

Eigen::Vector3d pos;
Eigen::Vector4d quat;

void extractPosAndQuat(const geometry_msgs::Point &pt, const geometry_msgs::Quaternion &q)
{
    pos(0) = pt.x; pos(1) = pt.y; pos(2) = pt.z;
    quat(0) = q.x; quat(1) = q.y; quat(2) = q.z; quat(3) = q.w;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    extractPosAndQuat(msg->pose.position, msg->pose.orientation);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    extractPosAndQuat(msg->pose.pose.position, msg->pose.pose.orientation);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "frame_add");
    ros::NodeHandle nh("~");

    int32_t _id;
    nh.getParam("id", _id);

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    ros::Subscriber pose_sub = nh.subscribe("/pose", 1000, poseCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1000, odomCallback);

    pos = Eigen::Vector3d::Zero();
    quat = Eigen::Vector4d::Zero();
    quat(3) = 1.0;

    ros::Rate rate(100.0);
    while (nh.ok()) {
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "uav_" + std::to_string(_id);
        transformStamped.transform.translation.x = pos(0);
        transformStamped.transform.translation.y = pos(1);
        transformStamped.transform.translation.z = pos(2);
        transformStamped.transform.rotation.x = quat(0);
        transformStamped.transform.rotation.y = quat(1);
        transformStamped.transform.rotation.z = quat(2);
        transformStamped.transform.rotation.w = quat(3);

        br.sendTransform(transformStamped);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}