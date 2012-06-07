#include <cstdlib>
#include <algorithm>
#include <ca_common/Trajectory.h>
#include <ca_common/math.h>
#include <mk_model/mk_common.h>
#include <trajectory_control/trajectory_control_lib.h>

using namespace CA;
using namespace std;


MkVelocityControlCommand TrajectoryControl::positionControl(double dt, State curr_state,  TrajectoryControlState &controlstate, Trajectory &path)
{
  MkVelocityControlCommand command;
  command.heading = curr_state.pose.orientation_rad[2];
  //Check precondition of inputs:
  if(!std::isfinite(dt) || !CA::isfinite(curr_state) || !path.isfinite())
    {
      ROS_ERROR_STREAM_THROTTLE(1, "Trajectory_control: Received invalid inputs stopping.");
      return command;
    }
  //Preconditions passed can proceed safely

  if(path.size() < 1)
    {
      ROS_INFO_STREAM_THROTTLE(10, "Trajectory_control: Path does not contain any waypoints, no command issued");
      return command;
    }

  bool isHovering, nearingEnd, sharpCorner;
  State closest_state = path.projectOnTrajInterp(curr_state, &isHovering,&controlstate.closestIdx);

  //This is the state we will control to. This looks ahead based on the current speed to account for control reaction delays.
  double speed = curr_state.rates.velocity_mps.norm();
  double lookaheadDist = std::max(0.3,pr.lookAhead * speed);
  State pursuit_state = path.lookAheadMaxAngle(controlstate.closestIdx, lookaheadDist,pr.lookAheadAngle, &nearingEnd);
  //Next we also look up if we need to slow down based on our maximum acceleration.
  double stoppingDistance =  math_tools::stoppingDistance(pr.deccelMax,pr.reactionTime,speed);
  double distanceToEnd = path.distanceToEnd(controlstate.closestIdx, 5.0 + stoppingDistance,pr.lookAheadAngle, &nearingEnd, &sharpCorner);//Added an offset to prevent problems at low speed

  if (nearingEnd)
    {

      double maxDesiredSpeed = math_tools::stoppingSpeed(pr.deccelMax,pr.reactionTime,distanceToEnd);

      if(maxDesiredSpeed < 0.5 || sharpCorner)
	{
	  double az = closest_state.pose.position_m[2];
	  closest_state = pursuit_state;
	  closest_state.pose.position_m[2] = az;
	}
      math_tools::normalize(pursuit_state.rates.velocity_mps);
      pursuit_state.rates.velocity_mps *=maxDesiredSpeed;
      
    }
  Vector3D desired_velocity = pursuit_state.rates.velocity_mps;

  Vector3D curr_to_closest = closest_state.pose.position_m - curr_state.pose.position_m;
  if(curr_to_closest.norm() > pr.trackingThreshold)
    {
      ROS_WARN_STREAM_THROTTLE(10, "Trajectory_control: Greater than 10 meters from path, no command issued");
      return command;
    }
  
  if(desired_velocity.norm() > pr.maxSpeed)
    {
      ROS_WARN_STREAM_THROTTLE(10, "Trajectory_control: Commanded path exceeds maximum allowed speed"<<desired_velocity.norm()<<" > "<<pr.maxSpeed);
      math_tools::normalize(desired_velocity);
      desired_velocity *= pr.maxSpeed;
    }
  
  //  ROS_INFO_STREAM(std::fixed<<"CP: "<<curr_state.pose.position_m<<" CS: "<<closest_state.pose.position_m<<" PP: "<<pursuit_state.pose.position_m);
  
  Vector3D path_tangent = desired_velocity;
  math_tools::normalize( path_tangent);
  Vector3D curr_to_path = curr_to_closest - path_tangent * math_tools::dot(path_tangent,curr_to_closest);
  Vector3D path_normal = curr_to_path;
  math_tools::normalize(path_normal);
  double cross_track_error = math_tools::dot(curr_to_closest, path_normal);
  double cross_track_error_d = cross_track_error - controlstate.prev_cross_track_error;

  
  controlstate.crossTrackIntegrator += cross_track_error;

  controlstate.crossTrackIntegrator  = math_tools::Limit(pr.crossTrackIMax,controlstate.crossTrackIntegrator);

  
  double u_cross_track = pr.crossTrackP * cross_track_error +
                         pr.crossTrackI * controlstate.crossTrackIntegrator +
                         pr.crossTrackD * cross_track_error_d/dt;
  

  
  controlstate.prev_cross_track_error = cross_track_error;

  //   ROS_INFO_STREAM("Desired vel: "<<desired_velocity<<" act: "<<curr_state.rates.velocity_mps);
   //  ROS_INFO_STREAM("Cross Track U" <<  u_cross_track);  

   //ROS_INFO_STREAM("Path Normal"  << path_normal );
   //ROS_INFO_STREAM("Path Tangent"  << path_tangent);
   
  Vector3D commandv = desired_velocity +  u_cross_track * path_normal;
  
  //Do separate terms for the z control:
  double z_trackerror =   curr_to_closest[2] - controlstate.prev_z_track_error;
  controlstate.prev_z_track_error = curr_to_closest[2];

  double ztrack =pr.crossTrackPZ * curr_to_closest[2] +
                         pr.crossTrackDZ * z_trackerror/dt;
  commandv[2] = desired_velocity[2] + ztrack;
  //  ROS_INFO_STREAM(dt<<" "<<commandv[2]<<" "<<desired_velocity[2]<<" "<<curr_to_closest[2]<<" "<<z_trackerror);
  if(commandv.norm() > pr.maxSpeed)
    {

      commandv *= (pr.maxSpeed/commandv.norm());
    }
  command.velocity = commandv;
  command.heading = pursuit_state.pose.orientation_rad[2];
  //Check output invariants:
  if(!command.isfinite())
    {
      ROS_ERROR_STREAM_THROTTLE(1, "Trajectory_control: Calculated invalid command. Stopping.");
      command.velocity[0] = 0;
      command.velocity[1] = 0;
      command.velocity[2] = 0;
      command.heading = 0.0;
      return command;
    }

  return command;
}


