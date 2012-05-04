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
    double alongTrackP;
    double alongTrackI;
    double alongTrackD;
    double crossTrackP;
    double crossTrackI;
    double crossTrackD;
    double loopRate;
    double maxSpeed;
    double lookAhead;
    double alongTrackIMax;
    double crossTrackIMax;
    double trackingThreshold;
    TrajectoryControlParameters():
      alongTrackP(0.0),
      alongTrackI(0.0),
      alongTrackD(0.0),
      crossTrackP(0.0),
      crossTrackI(0.0),
      crossTrackD(5.0),
      loopRate(5.0),
      maxSpeed(5.0),
      lookAhead(0.0),
      alongTrackIMax(0.0),
      crossTrackIMax(5.0),
      trackingThreshold(10.)
    {
    }
    bool loadParameters(ros::NodeHandle &n)
    {
      bool found = true;
      found = found && n.getParam("alongTrackP", alongTrackP);
      found = found && n.getParam("alongTrackI", alongTrackI);
      found = found && n.getParam("alongTrackD", alongTrackD);
      found = found && n.getParam("alongTrackIMax", alongTrackIMax);
     
      found = found && n.getParam("crossTrackP", crossTrackP);
      found = found && n.getParam("crossTrackI", crossTrackI);
      found = found && n.getParam("crossTrackD", crossTrackD);
      found = found && n.getParam("crossTrackIMax", crossTrackIMax);
      found = found && n.getParam("loopRate", loopRate);
      found = found && n.getParam("maxSpeed", maxSpeed);
      found = found && n.getParam("lookAhead", lookAhead);
      found = found && n.getParam("trackingThreshold", lookAhead);
      return found;
    }
  };
  

  class TrajectoryControl
  {
  public:
    TrajectoryControlParameters pr;
    TrajectoryControl(TrajectoryControlParameters para=TrajectoryControlParameters()):
      pr(para)
    {}
    MkVelocityControlCommand positionControl(double dt, CA::State curr_state,  CA::TrajectoryControlState &controlstate, CA::Trajectory &path);
  };

}

#endif
