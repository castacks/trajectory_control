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

  if(path.t.size() < 1)
    {
      ROS_INFO_STREAM_THROTTLE(10, "Trajectory_control: Path does not contain any waypoints, no command issued");
      return command;
    }
  bool isHovering, nearingEnd;
  State closest_state = path.projectOnTrajInterp(curr_state, &isHovering,&controlstate.closestIdx);
  State pursuit_state = path.lookAhead(controlstate.closestIdx, pr.lookAhead, &nearingEnd);
  ROS_INFO_STREAM("ps: "<<pursuit_state<<" at end" << nearingEnd<<" "<<closest_state<<" at end "<<isHovering);
  if (nearingEnd)
    {
      ROS_INFO_STREAM_THROTTLE(10,"Nearing End.");
      pursuit_state = closest_state;
    }
  if(isHovering)
    {
      ROS_INFO_STREAM_THROTTLE(10,"At End.");
      closest_state = path.t[path.t.size()-1];
      closest_state.rates.velocity_mps[0] = 0;
      closest_state.rates.velocity_mps[1] = 0;
      closest_state.rates.velocity_mps[2] = 0;
      pursuit_state = closest_state;
    }  
  Vector3D desired_velocity = pursuit_state.rates.velocity_mps;
   ROS_INFO_STREAM("dv af: "<<desired_velocity);
   //  Vector3D curr_to_pure    = pursuit_state.pose.position_m - curr_state.pose.position_m;
  Vector3D curr_to_closest = closest_state.pose.position_m - curr_state.pose.position_m;
  if(curr_to_closest.norm() > pr.trackingThreshold)
    {
      ROS_WARN_STREAM_THROTTLE(10, "Trajectory_control: Greater than 10 meters from path, no command issued");
      return command;
    }
  
  if(desired_velocity.norm() > pr.maxSpeed)
    {
      ROS_WARN_STREAM_THROTTLE(10, "Trajectory_control: Commanded path exceeds maximum allowed speed");
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
  double along_track_error_d = desired_velocity.norm() - math_tools::dot(curr_state.rates.velocity_mps , path_tangent);
  
  controlstate.crossTrackIntegrator += cross_track_error;
  controlstate.alongTrackIntegrator += along_track_error_d;
  controlstate.crossTrackIntegrator  = math_tools::Limit(pr.crossTrackIMax,controlstate.crossTrackIntegrator);
  controlstate.alongTrackIntegrator  = math_tools::Limit(pr.alongTrackIMax,controlstate.alongTrackIntegrator);
  
  double u_cross_track = pr.crossTrackP * cross_track_error +
                         pr.crossTrackI * controlstate.crossTrackIntegrator +
                         pr.crossTrackD * cross_track_error_d;
  

  
  double u_along_track = pr.alongTrackP * along_track_error_d +
                         pr.alongTrackI * controlstate.alongTrackIntegrator +
                         pr.alongTrackD * (along_track_error_d - controlstate.prev_along_track_error);
  
  controlstate.prev_cross_track_error = cross_track_error;
  controlstate.prev_along_track_error = along_track_error_d;
  //   ROS_INFO_STREAM("Desired vel: "<<desired_velocity<<" act: "<<curr_state.rates.velocity_mps);
   //  ROS_INFO_STREAM("Cross Track U" <<  u_cross_track);  
   //ROS_INFO_STREAM("Along Track U" << u_along_track);
   //ROS_INFO_STREAM("Path Normal"  << path_normal );
   //ROS_INFO_STREAM("Path Tangent"  << path_tangent);
   
  Vector3D commandv = desired_velocity +
                     u_cross_track * path_normal +
                     u_along_track * path_tangent;
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


