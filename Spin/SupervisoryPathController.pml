/* Vehicle state */
byte speed = 0;
short egoPosition = 0;

/* Supervisor state */
int state = 0;
#define FIND_SPP 0
#define FIND_FIRST_UPP 1
#define EXECUTE_FIRST_UPP 2
#define UPP_TO_SPP 3
#define STATE_SPP 4
#define SPP_TO_UPP 5
#define EXECUTE_SECOND_UPP 6
#define FINISH_UPP 7
#define GOAL_POSITION 8
#define WAITING_FOR_GOAL 9
#define WAIT_FOR_TRANSFORMS 10
#define CALL_FOR_PLAN 11
#define PARKING_TO_PARKING 12
#define FROM_PARKING 13
#define TRANSITION_TO_SPP 14
#define WAIT_TO_LEAVE_TP1 15
#define CLOSE_TO_TRANSITION2 16
#define TRANSITION_TO_UPP2 17
#define WAIT_TO_LEAVE_TP2 18
#define GO_TO_GOAL 19
#define EXECUTE_SSMP 100
#define EMERGENCY_STOP 101
#define SAFE_STOP 102

mtype = {initPose, goalPose, tPoint1, tPoint2, success, fail, plan, UPP, SPP, STITCH, GOAL};

/*
  Used when sending commands to the trajectory planner.
  The number of the type of the command
  The position when transition into next state
  The length of the current path
  The minumum allowed speed
  The maximum allowed speed
*/
typedef PlannerCommand {
  short nextTransition;
  short pathLength;
};

/* Communication channels between different nodes. */
/* To the first two planners we send plan(startPosition, endPosition) */
chan supervisorToSPP = [1] of {mtype, short, short};
chan supervisorToUPP = [1] of {mtype, short, short};
/* The SPP planner returns success/fail and transition points */
chan SPPToSupervisor = [1] of {mtype, short, short, bool, bool};
/* UPP just sends success/fail response */
chan UPPToSupervisor = [1] of {mtype};
/* SSMP is told to plan, it answers with success/fail */
chan supervisorToSSMP = [1] of {mtype};
chan SSMPToSupervisor = [1] of {mtype};

/* The trajectory planner gets (UPP/STITCH/SPP/GOAL, PlannerCommand) */ 
chan supervisorToTrajectory = [1] of {mtype, PlannerCommand};
/* The trajectory planner answers with success when the transition point is reached. */
chan trajectoryToSupervisor = [1] of {mtype};

/* (threshold, horizon, minSpd, maxSpd) */
chan trajectoryToControl = [1] of {short, short};
/* success/fail */
chan controlToTrajectory = [1] of {mtype};

/* Status messages from planners */
bool SPP_success;
bool first_UPP_success;
bool second_UPP_success;
bool SSMP_success;
bool SPP_requested;
bool first_UPP_requested;
bool second_UPP_requested;
bool SSMP_requested;
bool controlFailure;
bool goal_received = true;
bool odom_init = true;
bool ego_pos_init = true;
bool newGoalReceived = false;

/* Info from planners */
short distanceToFirstTransitionPoint = 500;
short distanceToSecondTransitionPoint = 1000;
short distanceToGoalPosition = 1500;

/* Parameters */
short distanceThreshold = 10;

/* External events */
bool sensorFailure = false;

/* Ghost variables for verification */
bool firstUPPknown = false, secondUPPknown = false, SPPknown = false;
byte nrActiveControlCommands = 0;

proctype SupervisoryPathController() {
/*
Arbitrates between SPP, UPP, SSMP, and AEB based on mode.
*/
mtype response;
short tP1, tP2;
bool tP1_exists, tP2_exists;
PlannerCommand command;

WaitingForGoal:
  state = WAITING_FOR_GOAL;
  if
    :: goal_received -> goto WaitForTransforms;
  fi;
  
WaitForTransforms:
  state = WAIT_FOR_TRANSFORMS;
  if
    :: odom_init && ego_pos_init -> goto CallForPlan;
  fi;
  
CallForPlan:
  atomic {
    SPP_requested = false;
    first_UPP_requested = false;
    second_UPP_requested = false;
    SSMP_requested = false;
    SPPknown = false;
    firstUPPknown = false;
    secondUPPknown = false;
    sensorFailure = false;
    state = CALL_FOR_PLAN;
  }
  
  supervisorToSPP ! plan(0, distanceToGoalPosition);
              
  SPPToSupervisor ? response, tP1, tP2, tP1_exists, tP2_exists;
  SPP_requested = true;
  
  if
    :: response == success && tP1_exists && tP2_exists ->
         SPPknown = true;
         goto ParkingToParking;
    :: response == fail ->
         goto ExecuteSsmp;
  fi;
  
ParkingToParking:
  state = PARKING_TO_PARKING;
  
  supervisorToUPP ! plan(0, tP1);
              
  UPPToSupervisor ? response;
  first_UPP_requested = true;
  
  if
    :: response == success ->
         firstUPPknown = true;
         goto FromParking
    :: response == fail ->
         goto ExecuteSsmp;
  fi;
  
FromParking:
  state = FROM_PARKING;
  
  command.nextTransition = tP1 - distanceThreshold;
  command.pathLength = tP1;
  supervisorToTrajectory ! UPP(command);
  
  if
    :: sensorFailure = true ->
         goto ExecuteSsmp;
    :: egoPosition > tP1 - distanceThreshold ->
         goto TransitionToSPP;
  fi;
  
TransitionToSPP:
  state = TRANSITION_TO_SPP;
  
  command.nextTransition = tP1;
  command.pathLength = tP2;
  supervisorToTrajectory ! SPP(command);
  
  if
    :: sensorFailure = true ->
         goto ExecuteSsmp;
    :: egoPosition > tP1 ->
         goto WaitToLeaveTp1;
  fi;
  
WaitToLeaveTp1:
  state = WAIT_TO_LEAVE_TP1;
  
  if
    :: sensorFailure = true ->
         goto ExecuteSsmp;
    :: egoPosition > tP1 + distanceThreshold && tP2_exists ->
         goto Spp;
  fi;
  
Spp:
  state = STATE_SPP;
  
  command.nextTransition = tP2 - distanceThreshold;
  command.pathLength = tP2;
  supervisorToTrajectory ! SPP(command);
  
  if
    :: sensorFailure = true ->
         goto ExecuteSsmp;
    :: egoPosition > tP2 - distanceThreshold ->
         goto CloseToTransition2;
  fi;
  
CloseToTransition2:
  state = CLOSE_TO_TRANSITION2;
  
  supervisorToUPP ! plan(tP2, distanceToGoalPosition);
              
  UPPToSupervisor ? response;
  second_UPP_requested = true;
  
  if
    :: sensorFailure = true ->
         goto ExecuteSsmp;
    :: response == success ->
         secondUPPknown = true;
         goto TransitionToUPP2;
    :: response == fail ->
         goto ExecuteSsmp;
  fi;
  
TransitionToUPP2:
  state = TRANSITION_TO_UPP2;
  
  command.nextTransition = tP2;
  command.pathLength = distanceToGoalPosition;
  supervisorToTrajectory ! UPP(command);
  
  if
    :: sensorFailure = true ->
         goto ExecuteSsmp;
    :: egoPosition > tP2 ->
         goto WaitToLeaveTp2;
  fi;
  
WaitToLeaveTp2:
  state = WAIT_TO_LEAVE_TP2;
  
  if
    :: sensorFailure = true ->
         goto ExecuteSsmp;
    :: egoPosition > tP2 + distanceThreshold ->
         goto GoToGoal;
  fi;
  
GoToGoal:
  state = GO_TO_GOAL;
  
  command.nextTransition = distanceToGoalPosition - distanceThreshold;
  command.pathLength = distanceToGoalPosition;
  supervisorToTrajectory ! UPP(command);
  
  if
    :: sensorFailure = true ->
         goto ExecuteSsmp;
    :: egoPosition > distanceToGoalPosition - distanceThreshold && speed == 0 ->
         goto StateGoal;
  fi;
  
StateGoal:
  state = GOAL_POSITION;
  if
    :: newGoalReceived = true ->
         speed == 0;
         egoPosition = 0;
         goto CallForPlan;
  fi;
endstate1:
  false;
  
ExecuteSsmp:
  state = EXECUTE_SSMP;
  
  supervisorToSSMP ! plan;
  SSMPToSupervisor ? response;
  SSMP_requested = true;
  
  if
    :: response == success ->
         goto SafeStop;
    :: response == fail ->
         goto EmergencyStop;
  fi;
  
SafeStop:
  state = SAFE_STOP;
endstate2:
  false;
  
EmergencyStop:
  state = EMERGENCY_STOP;
endstate3:
  false;

}

proctype StructuredAreaPathPlanner() {
/*
Gets a request of planning from a start position to a goal position.
Returns the transition points to unstructured areas.
Returns whether the path was created or not. Returns the path.
*/

short initPos;
short goalPos;
mtype response;
short tP1;
short tP2;
bool tP1_exists;
bool tP2_exists;

do
  :: supervisorToSPP ? plan, initPos, goalPos ->
       atomic {
         SPP_requested = true;
         if
           :: response = success -> 
             tP1 = distanceToFirstTransitionPoint;
             tP2 = distanceToSecondTransitionPoint;
             tP1_exists = true;
             tP2_exists = true;
             SPP_success = true;
           :: response = fail -> 
             tP1 = initPose;
             tP2 = initPose;
             SPP_success = false;
             tP1_exists = false;
             tP2_exists = false;
         fi;
       }
       SPPToSupervisor ! response, tP1, tP2, tP1_exists, tP2_exists;
od;

}

proctype UnstructuredAreaPathPlanner() {
/*
Gets a request of planning from a start position to a transition/goal position.
Returns whether the path was created or not. Returns the path.
*/

short initPos;
short endPos;
mtype response;

do
  :: supervisorToUPP ? plan, initPos, endPos ->
       atomic {
         if
           :: response = success;
           :: response = fail;
         fi;
         first_UPP_success = false;
         second_UPP_success = false;
         if
           :: initPos == 0 -> first_UPP_requested = true; first_UPP_success = response == success;
           :: initPos == distanceToSecondTransitionPoint -> second_UPP_requested = true; second_UPP_success = response == success;
           :: else -> skip;
         fi;
       }
       UPPToSupervisor ! response;
od;

}

proctype SafeStopManeuverPlanner() {
/*
Gets a request of planning from current position.
Creates a trajectory for a safe maneuver.
Returns whether trajectory planning succeeded.
Sends the trajectory to safety low level control.
*/

mtype response;

do
  :: supervisorToSSMP ? plan ->
       atomic {
         SSMP_requested = true;
         if
           :: response = success; SSMP_success = true;
           :: response = fail; SSMP_success = false;
         fi;
       }
       SSMPToSupervisor ! response;
od;

}

proctype EmergencyStopPlanner() {
/*
Gets a request of an emergency stop.
Follows the last active trajectory and brakes to stop as soon as possible.
*/

skip
}

proctype NominalTrajectoryPlanner() {
/*
Gets a path from the supervisor.
Based on the path, a trajectory is created and executed.
Event updates are communicated back to the supervisor.
*/

short nextThreshold = distanceToGoalPosition;
short nextHorizon = 0;
PlannerCommand command;

/* Speed controller */
do
  :: supervisorToTrajectory ? (_, command) ->
       nextThreshold = command.nextTransition;
       nextHorizon = command.pathLength;
       
       trajectoryToControl ! nextThreshold, nextHorizon;
       
       if
         :: true -> skip;
         :: controlFailure = true;
       fi;
od;

}

proctype NominalTrajectoryControl() {
/*
Gets a trajectory, which is executed by low level controllers.
*/

short lastPosition = egoPosition;
short threshold;
short horizon;
short lastHorizon = horizon;
byte minSpd;
byte maxSpd;
bool accelerationEnabled;

do
  :: trajectoryToControl ? threshold, horizon -> 
       atomic {
         if
           :: lastHorizon != horizon -> nrActiveControlCommands++; accelerationEnabled = true;
           :: else; skip;
         fi;
         lastHorizon = horizon;
       }
  :: empty(trajectoryToControl) && accelerationEnabled ->
       atomic {
         if
           :: horizon - egoPosition >= 630 -> speed = 35
           :: horizon - egoPosition >= 210 -> speed = 20
           :: horizon - egoPosition >= 55 -> speed = 10
           :: horizon - egoPosition >= 1 -> speed = 1
           :: else -> speed = 0; accelerationEnabled = false;
         fi;
         
         lastPosition = egoPosition;
         egoPosition = lastPosition + speed;
       }
od;

}

proctype SafetyTrajectoryControl() {
/*
Gets a safety critical trajectory, which is executed by low level controllers.
*/

skip
}

init {

run NominalTrajectoryControl();
run SupervisoryPathController();
run StructuredAreaPathPlanner();
run UnstructuredAreaPathPlanner();
run SafeStopManeuverPlanner();
run NominalTrajectoryPlanner();

}

/* We shall always be able to reach one of the end states. Otherwise we're in a deadlock. */
/* Liveness */
ltl noDeadlocks { []<>(state == GOAL_POSITION) || <>[](state == EMERGENCY_STOP) || <>[](state == SAFE_STOP) }

/* At some point in time, we shall stop and be stationary forever. */
/* Liveness */
ltl stopInTheEnd { []<>(speed == 0) }

/* We cannot reach the goal position if we don't have the paths. */
/* Safety */
ltl allPathsKnown { [](state == GOAL_POSITION -> (SPPknown && firstUPPknown && secondUPPknown) ) }

/* We don't want to drive outside the bounds of our paths. That might be unsafe. */
/* Safety */
ltl driveOnlyOnPaths { 
  []( 
        (egoPosition > 0 -> firstUPPknown)
     && (egoPosition > distanceToFirstTransitionPoint -> SPPknown)
     && (egoPosition > distanceToSecondTransitionPoint -> secondUPPknown)
     && (egoPosition <= distanceToGoalPosition + 1)
    )
}

/* Sensor error should mean safe or unsafe stop. */
/* Safety */
ltl safeStop { [](state == SAFE_STOP -> (
                    (
                          sensorFailure
                      ||  controlFailure
                      || (!SPP_success && SPP_requested)
                      || (!first_UPP_success && first_UPP_requested)
                      || (!second_UPP_success && second_UPP_requested)
                    )
                 && (SSMP_success && SSMP_requested))) }
                 
ltl unsafeStop { [](state == EMERGENCY_STOP -> ((
                       sensorFailure
                   ||  controlFailure
                   || (!SPP_success && SPP_requested)
                   || (!first_UPP_success && first_UPP_requested)
                   || (!second_UPP_success && second_UPP_requested))
                   && (!SSMP_success && SSMP_requested))) }
                   
ltl failure { []((sensorFailure && !(egoPosition >= distanceToGoalPosition)) -> <>(state == SAFE_STOP || state == EMERGENCY_STOP)) }

/*
This formula should fail.
It states that there is no way of getting to the goal state.
If this fails, Spin provides a trail that takes the system to the goal state.
If this does not fail, the vehicle can never reach the goal, and there is no reason to go on a mission.
*/
ltl failToReachGoal { [](state != GOAL_POSITION) }