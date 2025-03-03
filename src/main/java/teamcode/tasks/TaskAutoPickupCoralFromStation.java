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

 package teamcode.tasks;

 import edu.wpi.first.math.util.Units;
import frclib.vision.FrcPhotonVision;
 import teamcode.Robot;
import teamcode.RobotParams;
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
 
     private final Robot robot;
     private final TrcEvent driveEvent;
     private final TrcEvent grabberEvent;
    //  private final TrcEvent event;
 
     private int aprilTagId = -1;
     private TrcPose2D aprilTagPose = null;
     private Double visionExpiredTime = null;
 
     /**
      * Constructor: Create an instance of the object.
      *
      * @param robot specifies the robot object that contains all the necessary subsystems.
      */
     public TaskAutoPickupCoralFromStation(Robot robot)
     {
         super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
         this.robot = robot;
         this.driveEvent = new TrcEvent(moduleName + ".event");
         this.grabberEvent = new TrcEvent(moduleName + ".event");
        //  this.event = new TrcEvent(moduleName + ".event");
     }   //TaskAutoPickupCoralFromStation
 
    /**
     * This method starts the auto-assist operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param useVision specifies true to use vision to find the coral, false otherwise.
     * @param inAuto specifies true if caller is autonomous, false if in teleop.
     * @param relocalize specifies true to relocalize robot position, false otherwise.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
     public void autoPickupCoral(
        String owner, boolean useVision, boolean inAuto, boolean relocalize, TrcEvent completionEvent)
     {
         TaskParams taskParams = new TaskParams(useVision, inAuto, relocalize);
         tracer.traceInfo(
            moduleName, "autoPickupCoral(taskParams=(" + taskParams + "), event=" + completionEvent + ")");
         startAutoTask(owner, State.START, taskParams, completionEvent);
     }   //autoPickupCoral
 
     //
     // Implement TrcAutoTask abstract methods.
     //
 
    /**
     * This method is called to acquire ownership of all subsystems involved in the auto task operation. This is
     * typically called before starting an auto task operation.
     *
     * @param owner specifies the owner to acquire the subsystem ownerships.
     * @return true if acquired all subsystems ownership, false otherwise. It releases all ownership if any acquire
     *         failed.
     */
    @Override
    protected boolean acquireSubsystemsOwnership(String owner)
    {
        return owner == null || robot.robotDrive.driveBase.acquireExclusiveAccess(owner);
    }   //acquireSubsystemsOwnership

    /**
     * This method is called to release ownership of all subsystems involved in the auto task operation. This is
     * typically called if the auto task operation is completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void releaseSubsystemsOwnership(String owner)
    {
        if (owner != null)
        {
            TrcOwnershipMgr ownershipMgr = TrcOwnershipMgr.getInstance();
            tracer.traceInfo(
                moduleName,
                "Releasing subsystem ownership on behalf of " + owner +
                "\n\trobotDrive=" + ownershipMgr.getOwner(robot.robotDrive.driveBase));
            robot.robotDrive.driveBase.releaseExclusiveAccess(owner);
        }
    }   //releaseSubsystemsOwnership

    /**
     * This method is called to stop all the subsystems. This is typically called if the auto task operation is
     * completed or canceled.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     */
    @Override
    protected void stopSubsystems(String owner)
    {
        tracer.traceInfo(moduleName, "Stopping subsystems.");
        robot.robotDrive.cancel(owner);
    }   //stopSubsystems

    /**
     * This methods is called periodically to run the auto-assist task.
     *
     * @param owner specifies the owner that acquired the subsystem ownerships.
     * @param params specifies the task parameters.
     * @param state specifies the current state of the task.
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode (e.g. Autonomous, TeleOp, Test).
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false if running the fast loop on the main robot thread.
     */
    @Override
    protected void runTaskState(
        String owner, Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                // Check if already have coral, flash red light + stop
                if (RobotParams.Preferences.useIntake)
                {
                    if (robot.coralGrabber.hasObject())
                    {
                        if(robot.ledIndicator != null)
                        {
                            // TODO: This needs to be rewritten to use red without calling into Photon
                            // robot.ledIndicator.setPhotonDetectedObject(null, null);
                        }
                        tracer.traceInfo(moduleName, "We already have an object. Going to DONE");
                        sm.setState(State.DONE);
                        break;
                    }
                }
                // Start moving subsystems into position to save time
                robot.elevatorArmTask.setCoralStationPickupPosition(owner, null);
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
                    tracer.traceInfo(moduleName, "***** Not using Vision. Going to TAKE_CORAL directly");
                    sm.setState(State.TAKE_CORAL);
                }
                break;

            case FIND_STATION_APRILTAG:
                // Look for Coral Station AprilTag and relocalize robot.
                FrcPhotonVision.DetectedObject object =
                    robot.photonVisionBack.getBestDetectedAprilTag(RobotParams.Game.APRILTAG_STATION);

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
                else if (visionExpiredTime == null)
                {
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    tracer.traceInfo(moduleName, "***** No AprilTag found.");
                    // If we are in TeleOp and we cannot see the Station, at least we can turn on the hopper intake.
                    // If we are in Auto and we cannot see the Station, approach using odometry as Human Player can adjust.
                    sm.setState(taskParams.inAuto? State.APPROACH_STATION: State.TAKE_CORAL);
                }
                break;

             case APPROACH_STATION:
                TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                TrcPose2D targetPose;
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.2);
                sm.addEvent(driveEvent);

                if (taskParams.useVision && aprilTagPose != null){
                    tracer.traceInfo(moduleName, "*****Using Vision to drive to AprilTag");
                    targetPose = aprilTagPose.clone();  
                    targetPose.x += robot.robotInfo.cam2.camXOffset; // This value will need to be measured.
                    // What is 23? If this is a constant put in in RobotParams with a Name april tag height?
                    targetPose.x -= Math.sin(Units.degreesToRadians(aprilTagPose.angle)) * 23;
                    targetPose.y -= Math.cos(Units.degreesToRadians(aprilTagPose.angle)) * 23;
                    
                    tracer.traceInfo(moduleName, "****** \nAprilTagPose= " + aprilTagPose);
                    tracer.traceInfo(
                        moduleName,
                        state + "***** Approaching Coral Station with Vision:\n\tRobotFieldPose=" + robotPose +
                        //"\n\tintermediatePose=" + intermediatePose +
                        "\n\ttargetPose=" + targetPose);
                        robot.robotDrive.purePursuitDrive.start(
                            owner, driveEvent, 2.0, true, // Tune this timeout
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, targetPose);

                    // For testing don't gto the next state
                    sm.waitForSingleEvent(driveEvent, State.DONE);//State.TAKE_CORAL);
                }
                else if (!taskParams.inAuto)
                {
                    tracer.traceInfo(owner, "We cannot accurately use Odometry to move to the Station. We are in TeleOp so we can start grabber");
                    sm.setState(State.TAKE_CORAL);
                }
                else
                {
                    tracer.traceInfo(owner, "We cannot accurately use Odometry to move to the Station... but we're doing it anyways.");
                    robot.robotDrive.purePursuitDrive.start(
                        owner, driveEvent, 2.0, false, // Tune this timeout
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration, 
                        // TODO: Use both coral stations
                        robot.robotInfo.profiledMaxDeceleration, robot.adjustPoseByAlliance(RobotParams.Game.PROCESSOR_CORAL_STATION_RED, null));
                    sm.setState(State.TAKE_CORAL);
                }
                // } else if(!taskParams.inAuto){
                //     tracer.traceInfo(moduleName, "****** Using Robot Position to drive to closest AprilTag");
                    
                //     targetPose = PhotonVision.getClosestAprilTagPose(robotPose);
                //     targetPose.x += robot.robotInfo.cam1.camXOffset; // This value will need to be measured.
                //     targetPose.angle = 0.0;
                //     intermediatePose = targetPose.clone();
                //     intermediatePose.y = targetPose.y;
                //     targetPose.x = 0.0;
                     
                //     tracer.traceInfo(
                //         moduleName,
                //         state + "***** Approaching Coral Station without Vision:\n\tRobotFieldPose=" + robotPose +
                //         "\n\tintermediatePose=" + intermediatePose +
                //         "\n\ttargetPose=" + targetPose);
                //     robot.robotDrive.purePursuitDrive.start(
                //         owner, driveEvent, 2.0, true,
                //         robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                //         robot.robotInfo.profiledMaxDeceleration, intermediatePose, targetPose);
                // } else{
                //     tracer.traceInfo(moduleName, "Not going to AprilTag, we are just going to run the logic to pickup an object");
                // }
                break;

            case TAKE_CORAL:
                // Code to bring elevator and grabber down to position to pickup coral
                if (!robot.coralGrabber.hasObject())
                {
                    if (robot.elevatorArmTask != null && robot.coralGrabber != null)
                    {
                        tracer.traceInfo(
                            moduleName, "***** Moving Elevator and Arm to pickup position for Coral Station coral");
                        // Been moved to State.START
                        // Why would you wait for the elvator, I mean if you have one, just go?
                        // robot.elevatorArmTask.setCoralStationPickupPositions(owner, null);
                        // sm.addEvent(event);
                        
                        robot.coralGrabber.autoIntake(moduleName, 0.0, grabberEvent, 2.0);
                        // sm.addEvent(grabberEvent);

                        // sm.waitForEvents(State.DONE);
                        sm.waitForSingleEvent(grabberEvent, State.DONE);
                    }
                }
                else
                {
                    tracer.traceInfo(moduleName, "We already have a Coral!");
                    sm.setState(State.DONE);
                }
                break;

            default:
            case DONE:
                // Stop task.
                stopAutoTask(true);
                break;
        }
    }   //runTaskState

}   //class TaskAutoPickupCoralFromStation
