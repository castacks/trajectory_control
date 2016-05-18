/*
* Copyright (c) 2016 Carnegie Mellon University, Author <basti@andrew.cmu.edu, gdubey@andrew.cmu.edu>
*
* For License information please see the LICENSE file in the root directory.
*
*/


#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Eigen>
#include <trajectory_control/Command.h>


ros::Publisher marker_pub;
visualization_msgs::Marker odom_marker;
Eigen::Quaterniond q_w_i;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

    q_w_i.x() = msg->pose.pose.orientation.x;
    q_w_i.y() = msg->pose.pose.orientation.y;
    q_w_i.z() = msg->pose.pose.orientation.z;
    q_w_i.w() = msg->pose.pose.orientation.w;

    //odom_marker.header = msg->header;
    odom_marker.header.stamp = msg->header.stamp;
    odom_marker.header.frame_id = msg->child_frame_id;


    odom_marker.ns = "odometry_feedback";
    odom_marker.id = 0;

    odom_marker.type = visualization_msgs::Marker::ARROW;
    odom_marker.action = visualization_msgs::Marker::ADD;

    odom_marker.scale.x = 0.05;
    odom_marker.scale.y = 0.1;

    odom_marker.color.r = 1.0f;
    odom_marker.color.g = 0.0f;
    odom_marker.color.b = 0.0f;
    odom_marker.color.a = 1.0;

    odom_marker.lifetime = ros::Duration();

    geometry_msgs::Point end;
    //	end.x = msg->pose.pose.position.x + msg->twist.twist.linear.x/2;
    //	end.y = msg->pose.pose.position.y + msg->twist.twist.linear.y/2;
    //	end.z = msg->pose.pose.position.z + msg->twist.twist.linear.z/2;
    //	odom_marker.points.push_back(msg->pose.pose.position);
    //		odom_marker.points.push_back(end);

    Eigen::Vector3d world_vel;
    world_vel(0) = msg->twist.twist.linear.x;
    world_vel(1) = msg->twist.twist.linear.y;
    world_vel(2) = msg->twist.twist.linear.z;

    if (world_vel.norm() > 1000)
    {
        world_vel = world_vel.normalized() * 1000;
    }

    //Eigen::Vector3d body_vel = q_w_i.inverse()*world_vel;
    Eigen::Vector3d body_vel = world_vel;

    end.x = body_vel(0);
    end.y = body_vel(1);
    end.z = body_vel(2);

    odom_marker.points.clear();
    odom_marker.points.push_back(geometry_msgs::Point());
    odom_marker.points.push_back(end);

    odom_marker.frame_locked = true;

    marker_pub.publish(odom_marker);

    return;


}


void setpointCallback(const trajectory_control::Command::ConstPtr& msg)
{
    visualization_msgs::Marker sp_marker;
    //sp_marker.header = msg->header;
    sp_marker.header.stamp = odom_marker.header.stamp;
    sp_marker.header.frame_id = odom_marker.header.frame_id;


    sp_marker.ns = "velocity_sp";
    sp_marker.id = 1;

    sp_marker.type = visualization_msgs::Marker::ARROW;
    sp_marker.action = visualization_msgs::Marker::ADD;

    sp_marker.scale.x = 0.05;
    sp_marker.scale.y = 0.1;

    sp_marker.color.r = 0.0f;
    sp_marker.color.g = 0.0f;
    sp_marker.color.b = 1.0f;
    sp_marker.color.a = 1.0;

    sp_marker.lifetime = ros::Duration();

    geometry_msgs::Point end;
    Eigen::Vector3d world_vel;
    world_vel(0) = msg->velocity.x;
    world_vel(1) = msg->velocity.y;
    world_vel(2) = msg->velocity.z;

    Eigen::Vector3d body_vel = q_w_i.inverse()*world_vel;

    end.x = body_vel(0);
    end.y = body_vel(1);
    end.z = body_vel(2);

    sp_marker.points.push_back(geometry_msgs::Point());
    sp_marker.points.push_back(end);

    sp_marker.frame_locked = true;

    marker_pub.publish(sp_marker);

    return;
}

void setpointCallbackGoverned(const geometry_msgs::Vector3::ConstPtr& msg)
{
    visualization_msgs::Marker sp_marker;
    //sp_marker.header = msg->header;
    sp_marker.header.stamp = odom_marker.header.stamp;
    sp_marker.header.frame_id = odom_marker.header.frame_id;


    sp_marker.ns = "governed_velocity_sp";
    sp_marker.id = 1;

    sp_marker.type = visualization_msgs::Marker::ARROW;
    sp_marker.action = visualization_msgs::Marker::ADD;

    sp_marker.scale.x = 0.08;
    sp_marker.scale.y = 0.1;

    sp_marker.color.r = 0.0f;
    sp_marker.color.g = 1.0f;
    sp_marker.color.b = 0.0f;
    sp_marker.color.a = 1.0;

    sp_marker.lifetime = ros::Duration();

    geometry_msgs::Point end;
    Eigen::Vector3d world_vel;
    world_vel(0) = msg->x;
    world_vel(1) = msg->y;
    world_vel(2) = msg->z;

    Eigen::Vector3d body_vel = q_w_i.inverse()*world_vel;

    end.x = body_vel(0);
    end.y = body_vel(1);
    end.z = body_vel(2);

    sp_marker.points.push_back(geometry_msgs::Point());
    sp_marker.points.push_back(end);

    sp_marker.frame_locked = true;

    marker_pub.publish(sp_marker);

    return;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_traj_controller");

    ros::NodeHandle n;

    ros::Subscriber vel_sub = n.subscribe<trajectory_control::Command>("/trajectory_control/command", 1, setpointCallback);
    ros::Subscriber velGov_sub = n.subscribe<geometry_msgs::Vector3>("/speedgovernor/velocity_out", 1, setpointCallbackGoverned);
    ros::Subscriber odometry_sub = n.subscribe<nav_msgs::Odometry>("/trajectory_control/odometry", 1, odometryCallback);
    marker_pub = n.advertise<visualization_msgs::Marker>("/trajectory_control/velocity_markers", 1);

    ros::spin();

    return 0;
}
