
/*
 * Copyright (c) 2025 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package teamcode.autotasks;

 import frclib.vision.FrcPhotonVision;
 import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.subsystems.AlgaeGrabber;
import teamcode.subsystems.CoralArm;
import teamcode.subsystems.Elevator;
import teamcode.vision.PhotonVision;
import trclib.pathdrive.TrcPose2D;
 import trclib.robotcore.TrcAutoTask;
 import trclib.robotcore.TrcEvent;
 import trclib.robotcore.TrcOwnershipMgr;
 import trclib.robotcore.TrcRobot;
 import trclib.robotcore.TrcTaskMgr;
 import trclib.timer.TrcTimer;
 
 /**
  * This class implements Auto Pickup Coral From Station task.
  */
 public class TaskAutoPickupCoralFromStation extends TrcAutoTask<TaskAutoPickupCoralFromStation.State>
 {
     private static final String moduleName = TaskAutoPickupCoralFromStation.class.getSimpleName();
 
     public enum State
     {
         START,
         FIND_STATION_APRILTAG,
         APPROACH_STATION,
         TAKE_CORAL,
         DONE
     }   //enum State
 
     private static class TaskParams
     {
         boolean useVision;
         boolean inAuto;
         boolean relocalize;
 
         TaskParams(boolean useVision, boolean inAuto, boolean relocalize)
         {
             this.useVision = useVision;
             this.inAuto = inAuto;
             this.relocalize = relocalize;
         }   //TaskParams
 
         public String toString()
         {
             return "useVision=" + useVision +
                    ",inAuto=" + inAuto +
                    ",relocalize" + relocalize;
         }   //toString
     }   //class TaskParams
 
     private final String ownerName;
     private final Robot robot;
     private final TrcEvent event;
     private final TrcEvent driveEvent;
     private final TrcEvent grabberEvent;
 
     private String currOwner = null;
     private int aprilTagId = -1;
     private TrcPose2D aprilTagPose = null;
     private Double visionExpiredTime = null;
 
     /**
      * Constructor: Create an instance of the object.
      *
      * @param ownerName specifies the owner name to take subsystem ownership, can be null if no ownership required.
      * @param robot specifies the robot object that contains all the necessary subsystems.
      */
     public TaskAutoPickupCoralFromStation(String ownerName, Robot robot)
     {
         super(moduleName, ownerName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
         this.ownerName = ownerName;
         this.robot = robot;
         this.driveEvent = new TrcEvent(moduleName + ".event");
         this.event = new TrcEvent(moduleName + ".event");
         this.grabberEvent = new TrcEvent(moduleName + ".event");
     }   //TaskAutoPickupCoralFromStation
 
     /**
      * This method starts the auto-assist operation.
      *
      * @param useVision specifies true to use vision to find the coral, false otherwise.
      * @param inAuto specifies true if caller is autonomous, false if in teleop.
      * @param relocalize specifies true to relocalize robot position, false otherwise.
      * @param completionEvent specifies the event to signal when done, can be null if none provided.
      */
     public void autoPickupCoral(boolean useVision, boolean inAuto, boolean relocalize, TrcEvent completionEvent)
     {
         TaskParams taskParams = new TaskParams(useVision, inAuto, relocalize);
         tracer.traceInfo(moduleName, "taskParams=(" + taskParams + "), event=" + completionEvent);
         startAutoTask(State.START, taskParams, completionEvent);
     }   //autoPickupCoral
 
     //
     // Implement TrcAutoTask abstract methods.
     //
 
     /**
      * This method is called by the super class to acquire ownership of all subsystems involved in the auto-assist
      * operation. This is typically done before starting an auto-assist operation.
      *
      * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
      *         failed.
      */
     @Override
     protected boolean acquireSubsystemsOwnership()
     {
         boolean success = ownerName == null ||
                           (robot.robotDrive.driveBase.acquireExclusiveAccess(ownerName));
 
         if (success)
         {
             currOwner = ownerName;
             tracer.traceInfo(moduleName, "Successfully acquired subsystem ownerships.");
         }
         else
         {
             TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
             tracer.traceWarn(
                 moduleName,
                 "Failed to acquire subsystem ownership (currOwner=" + currOwner +
                 ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
             releaseSubsystemsOwnership();
         }
 
         return success;
     }   //acquireSubsystemsOwnership
 
     /**
      * This method is called by the super class to release ownership of all subsystems involved in the auto-assist
      * operation. This is typically done if the auto-assist operation is completed or canceled.
      */
     @Override
     protected void releaseSubsystemsOwnership()
     {
         if (ownerName != null)
         {
             TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
             tracer.traceInfo(
                 moduleName,
                 "Releasing subsystem ownership (currOwner=" + currOwner +
                 ", robotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase) + ").");
             robot.robotDrive.driveBase.releaseExclusiveAccess(currOwner);
             currOwner = null;
         }
     }   //releaseSubsystemsOwnership
 
     /**
      * This method is called by the super class to stop all the subsystems.
      */
     @Override
     protected void stopSubsystems()
     {
         tracer.traceInfo(moduleName, "Stopping subsystems.");
         robot.robotDrive.cancel(currOwner);
     }   //stopSubsystems
 
     /**
      * This methods is called periodically to run the auto-assist task.
      *
      * @param params specifies the task parameters.
      * @param state specifies the current state of the task.
      * @param taskType specifies the type of task being run.
      * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
      * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
      *        false if running the fast loop on the main robot thread.
      */
     @Override
     protected void runTaskState(
         Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
     {
         TaskParams taskParams = (TaskParams) params;
 
         switch (state)
         {
             case START:
                // Check if already have coral, flash red light + stop
                if(RobotParams.Preferences.useIntake)
                {
                    if(robot.intake.hasObject())
                    {
                        if(robot.ledIndicator != null)
                        {
                            robot.ledIndicator.setPhotonDetectedObject(null, null);
                        }
                        sm.setState(State.DONE);
                    }
                }
                 // Navigate robot to Coral Station point.
                 // Set up vision and subsystems according to task params.
                 aprilTagId = -1;
                 aprilTagPose = null;
                 if (taskParams.useVision && robot.photonVisionBack != null)
                 {
                     tracer.traceInfo(moduleName, "***** Moving to Coral Station using Vision.");
                     visionExpiredTime = null;
                     sm.setState(State.FIND_STATION_APRILTAG);
                 }
                 else
                 {
                     tracer.traceInfo(moduleName, "***** Not using Vision to move to Coral Station.");
                     sm.setState(State.APPROACH_STATION);
                 }
                 break;
 
             case FIND_STATION_APRILTAG:
                 // Look for Coral Station AprilTag and relocalize robot.
                 FrcPhotonVision.DetectedObject object =
                     robot.photonVisionBack.getBestDetectedAprilTag(new int[] {1, 2, 12, 13});
 
                 if (object != null)
                 {
                     aprilTagId = object.target.getFiducialId();
                     tracer.traceInfo(
                         moduleName, "***** Vision found AprilTag " + aprilTagId +
                         ": aprilTagPose=" + object.targetPose);
                     aprilTagPose = object.getObjectPose();
 
                     if (taskParams.relocalize)
                     {
                         robot.relocalizeRobotByAprilTag(object);
                     }
                     sm.setState(State.APPROACH_STATION);
                 }
                 else if (visionExpiredTime != null)
                 {
                     visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                 }
                 else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                 {
                     tracer.traceInfo(moduleName, "***** No AprilTag found.");
                     // If we are in TeleOp and we cannot see the Station, at least we can turn on the hopper intake.
                     // If we are in Auto and we cannot see the Station, quit.
                     sm.setState(taskParams.inAuto? State.DONE: State.TAKE_CORAL);
                 }
                 break;
 
             case APPROACH_STATION:
                TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                TrcPose2D targetPose, intermediatePose;
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.2);
                sm.addEvent(driveEvent);

                if(taskParams.useVision && aprilTagPose != null){
                    tracer.traceInfo(moduleName, "*****Using Vision to drive to AprilTag");
                    targetPose = aprilTagPose.clone();  
                    targetPose.x += robot.robotInfo.cam1.camXOffset; // This value will need to be measured.
                    targetPose.angle = 0.0;

                    intermediatePose = aprilTagPose.clone();
                    intermediatePose.y = targetPose.y;
                     
                    tracer.traceInfo(
                        moduleName,
                        state + "***** Approaching Coral Station with Vision:\n\tRobotFieldPose=" + robotPose +
                        "\n\tintermediatePose=" + intermediatePose +
                        "\n\ttargetPose=" + targetPose);
                        robot.robotDrive.purePursuitDrive.start(
                            currOwner, driveEvent, 2.0, true,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, intermediatePose, targetPose);

                } else if(!taskParams.inAuto){
                    tracer.traceInfo(moduleName, "****** Using Robot Position to drive to closest AprilTag");
                    
                    targetPose = PhotonVision.getClosestAprilTagPose(robotPose);
                    targetPose.x += robot.robotInfo.cam1.camXOffset; // This value will need to be measured.
                    targetPose.angle = 0.0;
                    intermediatePose = targetPose.clone();
                    intermediatePose.y = targetPose.y;
                    targetPose.x = 0.0;
                     
                    tracer.traceInfo(
                        moduleName,
                        state + "***** Approaching Coral Station without Vision:\n\tRobotFieldPose=" + robotPose +
                        "\n\tintermediatePose=" + intermediatePose +
                        "\n\ttargetPose=" + targetPose);
                    robot.robotDrive.purePursuitDrive.start(
                        currOwner, driveEvent, 2.0, true,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        robot.robotInfo.profiledMaxDeceleration, intermediatePose, targetPose);
                } else{
                    tracer.traceInfo(moduleName, "Not going to AprilTag, we are just going to run the logic to pickup an object");
                }
                sm.waitForEvents(State.TAKE_CORAL);
                break;


            case TAKE_CORAL:
                // Code to bring elevator and grabber down to position to pickup coral
                // TODO: We will need to add code for an additional sensor that will be in the hopper that will track whether an object has entered the hopper yet
                if(!robot.algaeGrabber.hasObject()){
                    double elevatorPos;
                    double armPos;
                    double finishDelay;
                    tracer.traceInfo(moduleName, "***** Moving Elevator and Arm to pickup position for Coral Station coral");
                    elevatorPos = Elevator.Params.HOPPER_PICKUP_POS;
                    armPos = CoralArm.Params.HOPPER_PICKUP_POS;
                    finishDelay = AlgaeGrabber.Params.FINISH_DELAY;
                    robot.moveSubsystem(currOwner, elevatorPos, 0.0, armPos, 0.0, 4.0, event);
                    sm.addEvent(event);

                    robot.algaeGrabber.autoIntake(currOwner, finishDelay, grabberEvent, 2.0);
                }

                sm.waitForEvents(State.DONE);
                 break;
             default:
             case DONE:
                 // Stop task.
                 stopAutoTask(true);
                 break;
         }
     }   //runTaskState
  
 }   //class TaskAutoPickupCoralFromStation
 