#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <cstdlib>
#include <algorithm>
#include <ca_common/Trajectory.h>
#include <ca_common/math.h>

CA::Trajectory path;
nav_msgs::Odometry odometry;

CA::Vector3D curr_position;
CA::Vector3D curr_velocity;
CA::State curr_state;

bool isHovering = false;
unsigned int currSeg;

void pathCallback(const ca_common::Trajectory::ConstPtr& msg)
{
  path.fromMsg(*msg);

  bool onEnd;
  CA::State closest_state = path.projectOnTrajInterp(curr_state, &onEnd);
  if(onEnd)
    isHovering = true;
  else
    isHovering = false;

  currSeg = 0;

  ROS_DEBUG_STREAM("Got path");
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
  odometry = *msg;

  curr_position[0] = odometry.pose.pose.position.x;
  curr_position[1] = odometry.pose.pose.position.y;
  curr_position[2] = odometry.pose.pose.position.z;

  curr_velocity = CA::msgc(odometry.twist.twist.linear);
  curr_state.pose.position_m = curr_position;

  ROS_DEBUG_STREAM("Got odometry");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_control");

  ros::NodeHandle n;
  double alongTrackP;
  double alongTrackI;

  double crossTrackP;
  double crossTrackI;
  double crossTrackD;

  double loopRate;

  double maxSpeed;
  double lookAhead;

  double alongTrackIntegrator = 0.0;
  double crossTrackIntegrator = 0.0;

  double alongTrackIMax;
  double crossTrackIMax;


  n.param("/trajectory_control/alongTrackP", alongTrackP, 0.0);
  n.param("/trajectory_control/alongTrackI", alongTrackI, 0.0);
  n.param("/trajectory_control/alongTrackIMax", alongTrackIMax, 0.0);

  n.param("/trajectory_control/crossTrackP", crossTrackP, 0.0);
  n.param("/trajectory_control/crossTrackI", crossTrackI, 0.0);
  n.param("/trajectory_control/crossTrackD", crossTrackD, 5.0);
  n.param("/trajectory_control/crossTrackIMax", crossTrackIMax, 5.0);

  n.param("/trajectory_control/loopRate", loopRate, 5.0);

  n.param("/trajectory_control/maxSpeed", maxSpeed, std::numeric_limits<double>::infinity());
  n.param("/trajectory_control/lookAhead", lookAhead, 0.0);

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3>("/trajectory_control/velocity", 100);
  ros::Publisher heading_pub = n.advertise<std_msgs::Float64>("/trajectory_control/heading", 100);

  ros::Subscriber path_sub = n.subscribe<ca_common::Trajectory>("/trajectory_control/path", 10, pathCallback);
  ros::Subscriber odometry_sub = n.subscribe<nav_msgs::Odometry>("/trajectory_control/odometry", 10, odometryCallback);
  
  ros::Rate loop_rate(loopRate);
  
  while (ros::ok())
  {
    loop_rate.sleep();
    ros::spinOnce();

    if((ros::Time::now() - odometry.header.stamp).toSec() > 1.0)
    {
      ROS_ERROR_STREAM_THROTTLE(1, "Trajectory_control: Odometry is more than one second old, no command issued");
      continue;
    }

    if(path.t.size() < 1)
    {
      ROS_ERROR_STREAM_THROTTLE(1, "Trajectory_control: Path does not contain any waypoints, no command issued");
      continue;
    }

    bool onEnd = true;
    double closestIdx = -1.0;
    CA::State closest_state;
    CA::State pursuit_state;

    while(onEnd && currSeg < path.t.size()-1)
    {
      CA::Trajectory segment;
      segment.t.push_back(path.t[currSeg]);
      segment.t.push_back(path.t[currSeg+1]);
      closest_state = segment.projectOnTrajInterp(curr_state, &onEnd, &closestIdx);

      closestIdx += currSeg;

      if(onEnd)
        currSeg++;
    }

    bool nearingEnd;
    //std::cout << "Idx: " << closestIdx << " lookAhead: " << lookAhead << std::endl;
    pursuit_state = path.lookAhead(closestIdx, lookAhead, &nearingEnd);
    //std::cout << "Pursuit State:" << std::endl << pursuit_state.pose.position_m << std::endl;

    if(onEnd)
      isHovering = true;

    if(isHovering)
    {
      closest_state = path.t[path.t.size()-1];
      closest_state.rates.velocity_mps *= 0;
      pursuit_state.rates.velocity_mps *= 0;
    }

    CA::Vector3D desired_velocity = pursuit_state.rates.velocity_mps;
    CA::Vector3D curr_to_closest = closest_state.pose.position_m - curr_position;

    if(curr_to_closest.norm() > 10.0)
    {
      ROS_ERROR_STREAM("Trajectory_control: Greater than 10 meters from path, no command issued");
      continue;
    }

    if(desired_velocity.norm() > maxSpeed)
    {
      ROS_WARN_STREAM("Trajectory_control: Commanded path exceeds maximum allowed speed");
      desired_velocity /= desired_velocity.norm();
      desired_velocity *= maxSpeed;
    }

    CA::Vector3D path_tangent = desired_velocity;
    if(path_tangent.norm() > 0.0)
      path_tangent.normalize();

    CA::Vector3D curr_to_path = curr_to_closest - path_tangent*CA::math_tools::dot(path_tangent,curr_to_closest);
    CA::Vector3D path_normal = curr_to_path;
    if(path_normal.norm() > 0.0)
      path_normal.normalize();

    double cross_track_error = CA::math_tools::dot(curr_to_closest,path_normal);
    double cross_track_error_d = -1.0*CA::math_tools::dot(curr_velocity,path_normal);
    double along_track_error_d = desired_velocity.norm() - CA::math_tools::dot(curr_velocity,path_tangent);

    crossTrackIntegrator += cross_track_error;
    alongTrackIntegrator += along_track_error_d;

    if(crossTrackIntegrator > crossTrackIMax)
      crossTrackIntegrator = crossTrackIMax;
    if(crossTrackIntegrator < -crossTrackIMax)
      crossTrackIntegrator = -crossTrackIMax;

    if(alongTrackIntegrator > alongTrackIMax)
      alongTrackIntegrator = alongTrackIMax;
    if(alongTrackIntegrator < -alongTrackIMax)
      alongTrackIntegrator = -alongTrackIMax;


    double u_cross_track = crossTrackP * cross_track_error +
                           crossTrackI * crossTrackIntegrator +
                           crossTrackD * cross_track_error_d;

    //std::cout << "Cross Track U" << std::endl << u_cross_track << std::endl;

    double u_along_track = alongTrackP * along_track_error_d +
                           alongTrackI * alongTrackIntegrator;

    //std::cout << "Along Track U" << std::endl << u_along_track << std::endl;

    //std::cout << "Path Normal" << std::endl << path_normal << std::endl;
    //std::cout << "Path Tangent" << std::endl << path_tangent << std::endl;


    CA::Vector3D command = desired_velocity +
                           u_cross_track*path_normal +
                           u_along_track*path_tangent;

    if(command.norm() > maxSpeed)
    {
      command /= command.norm();
      command *= maxSpeed;
    }

    geometry_msgs::Vector3 velocity_msg;
    velocity_msg = CA::msgc(command);

    std_msgs::Float64 heading_msg;
    heading_msg.data = pursuit_state.pose.orientation_rad[2];

    //std::cout << "Commanded Velocity:" << std::endl << velocity_msg << std::endl;

    vel_pub.publish(velocity_msg);
    heading_pub.publish(heading_msg);
  }

  return 0;
}


