#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <cstdlib>
#include <algorithm>
#include <ca_common/Trajectory.h>
#include <ca_common/math.h>
#include <visualization_msgs/Marker.h>
#include <mk_model/mk_common.h>
#include <trajectory_control/trajectory_control_lib.h>
#include <trajectory_control/Command.h>
#include <diagnostic_status_refiner/diagnostic_status_refiner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

using namespace CA;

//visualization_msgs::Marker odom_marker;
//ros::Publisher marker_pub;

ca::DiagStatusRefiner* refiner;
CA::Trajectory path;


CA::Vector3D curr_position;
CA::Vector3D curr_velocity;
CA::State curr_state;
ca_common::TrajectoryPoint hover_traj_point;
bool hover;

TrajectoryControlState controllerState;
ros::Time lastPlan;
boost::shared_ptr<diagnostic_updater::TopicDiagnostic> odomFreqDiag;

void pathCallback(const ca_common::Trajectory::ConstPtr& msg);
void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg);
void visualizeState(CA::State state, Eigen::Vector3d color, std::string frame_id, std::string ns);

void pathCallback(const ca_common::Trajectory::ConstPtr& msg)
{
    lastPlan = msg->header.stamp;
    path.fromMsg(*msg);
    controllerState.closestIdx = 0.0;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    curr_state =msgc(*msg);
    if(odomFreqDiag) odomFreqDiag->tick(msg->header.stamp);
}

void setHoverCallback(const std_msgs::String::ConstPtr & msg)
{
    ca_common::Trajectory emptyMsg;
    path.fromMsg(emptyMsg);
    std::string cmd = msg->data;
    if(cmd.compare("position")==0){
        hover_traj_point.position.x = curr_state.pose.position_m[0];
        hover_traj_point.position.y = curr_state.pose.position_m[1];
        hover_traj_point.position.z = curr_state.pose.position_m[2];
        hover_traj_point.heading = curr_state.pose.orientation_rad[2];
        hover = true;
        return;
    }
    hover = false;
    return;
}

void errorDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& status){
  refiner->AggregateEntries(status);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_control");
    ros::NodeHandle n("trajectory_control");
    ros::NodeHandle private_nh("~");
    double trajectory_expiration_time;
    private_nh.param<double>("trajectory_expiration_time",trajectory_expiration_time,1.0);
    TrajectoryControlParameters  parameters;
    if(!parameters.loadParameters(n))
    {
        ROS_ERROR_STREAM("Failed to load all parameters");
        return -1;
    }
    TrajectoryControl controller(parameters);
    controller.debugInit(n);
    ros::Publisher command_pub = n.advertise<trajectory_control::Command>("command", 100);
    ros::Publisher twist_command_pub = n.advertise<geometry_msgs::TwistStamped>("/command/twist", 100);
    ros::Publisher pubLookAheadState = n.advertise<nav_msgs::Odometry>("lookaheadpose", 100);
    nav_msgs::Odometry lookaheadpose;
    State lookheadstate;

  //  marker_pub = n.advertise<visualization_msgs::Marker>("goal_markers", 1);
    ros::TransportHints hints = ros::TransportHints().udp().tcpNoDelay();
    ros::Subscriber path_sub = n.subscribe<ca_common::Trajectory>("path", 10, pathCallback);
    ros::Subscriber odometry_sub = n.subscribe<nav_msgs::Odometry>("odometry", 10, odometryCallback, hints);
    ros::Subscriber hover_sub = n.subscribe<std_msgs::String>("hover", 10, setHoverCallback);

    ros::Rate loop_rate(parameters.loopRate);
    //CA::PetWatchdog pet;
    //if(!pet.setup(n))
    //{
    //    ROS_ERROR_STREAM("Was not able to setup watchdog");
        //return -1;
    //}

    //Diagnostics
    ca::DiagHelperClass errors;
    errors.updater_->setHardwareID("traj_control");
    errors.updater_->add("Trajectory Control Loop", errorDiagnostics);
    refiner = new ca::DiagStatusRefiner("errors");
    ros::Timer diag_timer = n.createTimer(ros::Duration(1.0), &ca::DiagHelperClass::callback,&errors);
    double target=100.0;
    const double period = 1.0 / target; 
    
    diagnostic_updater::FrequencyStatusParam freqParam(&target, &target, 0.1, 10);
    diagnostic_updater::TimeStampStatusParam timeParam(0, period *2);
    odomFreqDiag.reset(new diagnostic_updater::TopicDiagnostic("odomfreq", 
                                                     *errors.updater_, 
                                                     freqParam,
                                                     timeParam));    
    std::string error_msg;

    double dt = 1/parameters.loopRate;
    trajectory_control::Command command;
    command.header.frame_id = "base_frame";
    command.header.seq=0;
    hover = false;
    while (ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        double timediff = ros::Time::now().toSec() - curr_state.time_s;
        if(timediff > 1.0)
        {
            error_msg= "Trajectory_control: Odometry is more than one second old, no command issued"+boost::to_string(timediff)+ " "+boost::to_string(curr_state.time_s);
            ROS_ERROR_STREAM_THROTTLE(1, "Trajectory_control: Odometry is more than one second old, no command issued"<<timediff<< " "<< curr_state.time_s);
            refiner->AddRefinerEntry("traj control loop", 1, error_msg);
            continue;
        }

        /*if(!std::isfinite(dt) || !isfinite(curr_state) || !path.isfinite())
            pet.fault();
        else
            pet.alive();*/

        if ((ros::Time::now() - lastPlan).toSec() > trajectory_expiration_time){
            ROS_ERROR_STREAM_THROTTLE(1, "Trajectory age: " << (ros::Time::now() - lastPlan).toSec());
            error_msg= boost::to_string((ros::Time::now() - lastPlan).toSec());
            refiner->AddRefinerEntry("Trajectory age: ", 2, error_msg); 
        }

        if(path.size() < 1 || (ros::Time::now() - lastPlan).toSec() > trajectory_expiration_time)
        {            
            ca_common::Trajectory hoverMsg;
            hoverMsg.header.stamp = ros::Time::now();
            hoverMsg.header.frame_id = "/world";
            ca_common::TrajectoryPoint hoverTrajPoint;
            if(!hover)
            {
                ROS_WARN_STREAM("Trajectory_control: Path does not contain any waypoints, default to velocity hold hover");
                error_msg="Path does not contain any waypoints, default to velocity hold hover";
                refiner->AddRefinerEntry("Trajectory_control: ", 2, error_msg); 
                hoverTrajPoint.position.x = curr_state.pose.position_m[0];
                hoverTrajPoint.position.y = curr_state.pose.position_m[1];
                hoverTrajPoint.position.z = curr_state.pose.position_m[2];
                hoverTrajPoint.heading = curr_state.pose.orientation_rad[2];
            }
            else
            {
                //ROS_WARN_STREAM("Trajectory_control: Path does not contain any waypoints, default to position hold hover");
                ROS_WARN_STREAM("Hovering At:"<<hover_traj_point.position.x<<"::"<<hover_traj_point.position.y<<"::"
                  <<hover_traj_point.position.z); 
                error_msg=boost::to_string(hover_traj_point.position.x)+"::"+boost::to_string(hover_traj_point.position.y)
                  +"::"+boost::to_string(hover_traj_point.position.z);
                refiner->AddRefinerEntry("Hovering At: ", 1, error_msg); 
                hoverTrajPoint.position = hover_traj_point.position;
                hoverTrajPoint.heading = hover_traj_point.heading;
            }

            hoverMsg.trajectory.push_back(hoverTrajPoint);

            path.fromMsg(hoverMsg);
            controllerState.closestIdx = 0.0;

        }


		//ROS_ERROR_STREAM("curr_state "<<curr_state<<" path "<<path<<" lookheadstate "<<lookheadstate);
        MkVelocityControlCommand commandres = controller.positionControl(dt,curr_state,controllerState,path,lookheadstate);
        command.header.stamp = ros::Time::now();
        command.header.seq++;
        command.velocity     = CA::msgc(commandres.velocity);
        command.acceleration = CA::msgc(commandres.acceleration);
        command.heading      = commandres.heading;
        command.headingrate  = commandres.headingrate;
        command_pub.publish(command);


        geometry_msgs::TwistStamped twist;
        twist.header = command.header;
        twist.twist.linear = command.velocity;
        twist.twist.angular.x=twist.twist.angular.y=0.0;
        twist.twist.angular.z=command.headingrate;
        twist_command_pub.publish(twist);

        lookaheadpose = msgc(lookheadstate);
        lookaheadpose.header = command.header;
        lookaheadpose.header.frame_id = "world";

        pubLookAheadState.publish(lookaheadpose);
    }

    return 0;
}



