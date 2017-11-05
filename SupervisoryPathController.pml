/* Vehicle state */
int speed = 0;
int acceleration = 0;

proctype SupervisoryPathController() {
/*
Arbitrates between SPP, UPP, SSMP, and AEB based on mode.
*/


}

proctype StructuredAreaPathPlanner() {
/*
Gets a request of planning from a start position to a goal position.
Returns the transition points to unstructured areas.
Returns whether the path was created or not. Stores the path.
Upon execution request, the stored path is executed.
*/


}

proctype UnstructuredAreaPathPlanner() {
/*
Gets a request of planning from a start position to a transition/goal position.
Returns whether the path was created or not. Stores the path.
Upon execution request, the stored path is executed.
*/


}

proctype SafeStopManeuverPlanner() {
/*
Gets a request of planning from current position.
Creates a trajectory for a safe maneuver.
Returns whether trajectory planning succeeded.
Sends the trajectory to safety low level control.
*/


}

proctype EmergencyStopPlanner() {
/*
Gets a request of an emergency stop.
Follows the last active trajectory and brakes to stop as soon as possible.
*/


}

proctype NominalTrajectoryPlanner() {
/*
Gets a path from one of the path planners.
Based on the path, a trajectory is created.
*/


}

proctype NominalTrajectoryControl() {
/*
Gets a trajectory, which is executed by low level controllers.
*/


}

proctype SafetyTrajectoryControl() {
/*
Gets a safety critical trajectory, which is executed by low level controllers.
*/


}