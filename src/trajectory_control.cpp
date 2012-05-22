#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <cstdlib>
#include <algorithm>
#include <ca_common/Trajectory.h>
#include <ca_common/math.h>
#include <visualization_msgs/Marker.h>
#include <mk_model/mk_common.h>
#include <trajectory_control/trajectory_control_lib.h>
#include <watchdog/watchdog.h>
using namespace CA;

visualization_msgs::Marker odom_marker;
ros::Publisher marker_pub;

CA::Trajectory path;


CA::Vector3D curr_position;
CA::Vector3D curr_velocity;
CA::State curr_state;
TrajectoryControlState controllerState;

void pathCallback(const ca_common::Trajectory::ConstPtr& msg);
void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg);
void visualizeState(CA::State state, Eigen::Vector3d color, std::string frame_id, std::string ns);


void pathCallback(const ca_common::Trajectory::ConstPtr& msg)
{
  path.fromMsg(*msg);
  controllerState.closestIdx = 0.0;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
  curr_state =msgc(*msg);
}




int main(int argc, char **argv)
{
	ros::init(argc, argv, "trajectory_control");
	ros::NodeHandle n("trajectory_control");
	TrajectoryControlParameters  parameters;
	if(!parameters.loadParameters(n))
	  {
	    ROS_ERROR_STREAM("Failed to load all parameters");
	    return -1;
	  }
	TrajectoryControl controller(parameters);
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3>("velocity", 100);
	ros::Publisher heading_pub = n.advertise<std_msgs::Float64>("heading", 100);
	marker_pub = n.advertise<visualization_msgs::Marker>("goal_markers", 1);
	ros::TransportHints hints = ros::TransportHints().udp().tcpNoDelay();
	ros::Subscriber path_sub = n.subscribe<ca_common::Trajectory>("path", 10, pathCallback);
	ros::Subscriber odometry_sub = n.subscribe<nav_msgs::Odometry>("odometry", 10, odometryCallback, hints);

	ros::Rate loop_rate(parameters.loopRate);
	CA::PetWatchdog pet;	
	if(!pet.setup(n))
	  {
	    ROS_ERROR_STREAM("Was not able to setup watchdog");
	    return -1;
	  }

	double dt = 1/parameters.loopRate;
	while (ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
		double timediff = ros::Time::now().toSec() - curr_state.time_s;
		if(timediff > 1.0)
		{
		  ROS_INFO_STREAM_THROTTLE(1, "Trajectory_control: Odometry is more than one second old, no command issued"<<timediff<< " "<< curr_state.time_s);
		  continue;
		}

		if(!std::isfinite(dt) || !isfinite(curr_state) || !path.isfinite())
		  pet.fault();
		else
		  pet.alive();
		if(path.t.size() < 1)
		{
			ROS_INFO_STREAM_THROTTLE(10, "Trajectory_control: Path does not contain any waypoints, no command issued");
			continue;
		}

		
		MkVelocityControlCommand command = controller.positionControl(dt,curr_state,controllerState,path);

		geometry_msgs::Vector3 velocity_msg;
		velocity_msg = CA::msgc(command.velocity);

		std_msgs::Float64 heading_msg;
		heading_msg.data = command.heading;

		//std::cout << "Commanded Velocity:" << std::endl << velocity_msg << std::endl;

	
		vel_pub.publish(velocity_msg);
		heading_pub.publish(heading_msg);
	}

	return 0;
}


