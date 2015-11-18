#ifndef _TRAJECTORY_CONTROL_LIB_H_
#define _TRAJECTORY_CONTROL_LIB_H_

#include <mk_model/mk_common.h>
#include <ca_common/math.h>
#include <ros/node_handle.h>


namespace CA
{


  class TrajectoryControlParameters
  {
  public:
    double crossTrackP;
    double crossTrackI;
    double crossTrackD;
    double crossTrackPZ;
    double crossTrackDZ;
    double alongTrackP;
    double headingRateP;
    double loopRate;
    double maxSpeed;
    double lookAhead;
    double lookAheadAngle;
    double crossTrackIMax;
    double trackingThreshold;
    double deccelMax;
    double reactionTime;
    TrajectoryControlParameters():
      crossTrackP(0.0),
      crossTrackI(0.0),
      crossTrackD(5.0),
      crossTrackPZ(0.0),
      crossTrackDZ(0.0),
      alongTrackP(0.0),
      headingRateP(0.0),
      loopRate(5.0),
      maxSpeed(5.0),
      lookAhead(0.5),
      lookAheadAngle(1.0),
      crossTrackIMax(5.0),
      trackingThreshold(10.),
      deccelMax(3.0),
      reactionTime(0.5)
    {
    }
    bool loadParameters(ros::NodeHandle &n)
    {
      bool found = true;

      found = found && n.getParam("crossTrackP", crossTrackP);
      found = found && n.getParam("crossTrackI", crossTrackI);
      found = found && n.getParam("crossTrackD", crossTrackD);
      found = found && n.getParam("crossTrackIMax", crossTrackIMax);

      found = found && n.getParam("crossTrackPZ", crossTrackPZ);
      found = found && n.getParam("crossTrackDZ", crossTrackDZ);

      found = found && n.getParam("alongTrackP", alongTrackP);

      found = found && n.getParam("loopRate", loopRate);
      found = found && n.getParam("maxSpeed", maxSpeed);
      found = found && n.getParam("lookAhead", lookAhead);
      found = found && n.getParam("lookAheadAngle", lookAheadAngle);
      found = found && n.getParam("trackingThreshold", trackingThreshold);
      found = found && n.getParam("deccelMax", deccelMax);
      found = found && n.getParam("reactionTime", reactionTime);
      found = found && n.getParam("headingRateP", headingRateP);
      return found;
    }
  };


  class TrajectoryControl
  {
  public:
    bool debug;
    TrajectoryControlParameters pr;
    ros::Publisher pubDebugVXY,pubDebugThrust;
    TrajectoryControl(TrajectoryControlParameters para=TrajectoryControlParameters()):
        debug(false),
        pr(para)
    {}
    void debugInit(ros::NodeHandle &n);
    MkVelocityControlCommand positionControl(double dt, CA::State curr_state,  CA::TrajectoryControlState &controlstate, CA::Trajectory &path, State &lookahead);
  };

}

#endif
