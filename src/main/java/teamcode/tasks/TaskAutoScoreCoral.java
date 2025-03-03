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
import teamcode.vision.PhotonVision.PipelineType;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcAutoTask;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcOwnershipMgr;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcTaskMgr;
import trclib.timer.TrcTimer;

/**
 * This class implements Auto Score Coral task.
 */
public class TaskAutoScoreCoral extends TrcAutoTask<TaskAutoScoreCoral.State>
{
    private static final String moduleName = TaskAutoScoreCoral.class.getSimpleName();

    public enum State
    {
        START,
        FIND_REEF_APRILTAG,
        APPROACH_REEF,
        SCORE_CORAL,
        DONE
    }   //enum State

    private static class TaskParams
    {
        boolean useVision;
        int reefLevel;  //0-3, with 0 being the trough and 3 being the high reef
        boolean removeAlgae;
        boolean inAuto;
        boolean relocalize;
        int scoreSide;

        TaskParams(
            boolean useVision, int reefLevel, boolean removeAlgae, boolean inAuto, boolean relocalize, int scoreSide)
        {
            this.useVision = useVision;
            this.reefLevel = reefLevel;
            this.removeAlgae = removeAlgae;
            this.inAuto = inAuto;
            this.relocalize = relocalize;
            this.scoreSide = scoreSide;
        }   //TaskParams

        public String toString()
        {
            return "useVision=" + useVision +
                   ",reefLevel=" + reefLevel +
                   ",removeAlgae=" + removeAlgae +
                   ",inAuto=" + inAuto +
                   ",relocalize=" + relocalize + 
                   ",scoreSide=" + scoreSide;
        }   //toString
    }   //class TaskParams

    private final Robot robot;
    private final TrcEvent driveEvent;
    private final TrcEvent elevatorArmEvent;
    private final TrcEvent scoreEvent;

    private int aprilTagId = -1;
    private TrcPose2D aprilTagPose = null;
    private Double visionExpiredTime;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object that contains all the necessary subsystems.
     */
    public TaskAutoScoreCoral(Robot robot)
    {
        super(moduleName, TrcTaskMgr.TaskType.POST_PERIODIC_TASK);
        this.robot = robot;
        this.driveEvent = new TrcEvent(moduleName + ".driveEvent");
        this.elevatorArmEvent = new TrcEvent(moduleName + ".elevatorArmEvent");
        this.scoreEvent = new TrcEvent(moduleName + ".scoreEvent");
    }   //TaskAutoScoreCoral

    /**
     * This method starts the auto task operation.
     *
     * @param owner specifies the owner to acquire subsystem ownerships, can be null if not requiring ownership.
     * @param useVision specifies true to use vision to find the coral, false otherwise.
     * @param reefLevel specifies the reef level to score the coral.
     * @param removeAlgae specifies true to remove algae from the reef, false otherwise.
     * @param inAuto specifies true if caller is autonomous, false if in teleop.
     * @param relocalize specifies true to relocalize robot position, false otherwise.
     * @param scoreSide specifies the reef branch to score the coral.
     * @param completionEvent specifies the event to signal when done, can be null if none provided.
     */
    public void autoScoreCoral(
        String owner, boolean useVision, int reefLevel, boolean removeAlgae, boolean inAuto, boolean relocalize,
        int scoreSide, TrcEvent completionEvent)
    {
        TaskParams taskParams = new TaskParams(useVision, reefLevel, removeAlgae, inAuto, relocalize, scoreSide);
        tracer.traceInfo(
            moduleName,
            "autoScoreCoral(owner=" + owner +
            ", taskParams=(" + taskParams +
            "), event=" + completionEvent + ")");
        startAutoTask(owner, State.START, taskParams, completionEvent);
    }   //autoScoreCoral

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
        // Restore to full power in case we have changed it.
        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
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
        String owner, Object params, State state, TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode,
        boolean slowPeriodicLoop)
    {
        TaskParams taskParams = (TaskParams) params;

        switch (state)
        {
            case START:
                // Set up vision and subsystems according to task params.
                aprilTagId = -1;
                aprilTagPose = null;
                if (taskParams.useVision && robot.photonVisionFront != null)
                {
                    tracer.traceInfo(moduleName, "***** Using AprilTag Vision.");
                    robot.photonVisionFront.setPipeline(PipelineType.APRILTAG);
                    visionExpiredTime = null;
                    sm.setState(State.FIND_REEF_APRILTAG);
                }
                else if (!taskParams.inAuto)
                {
                    // If we are not using vision, assume the driver has aligned the robot to the proper position
                    // and orientation for scoring. Then, we just score.
                    tracer.traceInfo(moduleName, "***** Not using AprilTag Vision.");
                    sm.setState(State.SCORE_CORAL);
                }
                else
                {
                    // If we are in auto, we must use vision. If not, quit.
                    sm.setState(State.DONE);
                }
                break;

            case FIND_REEF_APRILTAG:
                // Look for Reef AprilTag and relocalize robot.
                FrcPhotonVision.DetectedObject object =
                    robot.photonVisionFront.getBestDetectedAprilTag(RobotParams.Game.APRILTAG_REEFS);
                if (object != null)
                {
                    aprilTagId = object.target.getFiducialId();
                    tracer.traceInfo(
                        moduleName, "***** Vision found AprilTag " + aprilTagId +
                        ": aprilTagPose=" + object.targetPose);
                    aprilTagPose = object.getObjectPose();

                    if(taskParams.relocalize)
                    {
                        robot.relocalizeRobotByAprilTag(object);
                    }
                    sm.setState(State.APPROACH_REEF);
                }
                else if (visionExpiredTime == null)
                {
                    visionExpiredTime = TrcTimer.getCurrentTime() + 1.0;
                }
                else if (TrcTimer.getCurrentTime() >= visionExpiredTime)
                {
                    tracer.traceInfo(moduleName, "***** No AprilTag found.");
                    // We cannot see the Station, quit.
                    sm.setState(State.DONE);
                }
                break;

            case APPROACH_REEF:
                // If we come here, we are using Vision and it sees the AprilTag.
                TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                TrcPose2D targetPose;
                robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.2);
                targetPose = aprilTagPose.clone();
                targetPose.x += robot.robotInfo.cam1.camXOffset;// This value will need to be measured.
                // Code Review: Please explain this code. What is these magic numbers 17 and 7.5?
                targetPose.x -= Math.sin(Units.degreesToRadians(aprilTagPose.angle)) * 17;
                targetPose.y -= Math.cos(Units.degreesToRadians(aprilTagPose.angle)) * 17;
                targetPose.x = taskParams.scoreSide == 1? targetPose.x + 7.5: targetPose.x - 7.5; 
                tracer.traceInfo(
                    moduleName,
                    "***** Approaching Reef with Vision: RobotFieldPose=" + robotPose +
                    ", targetPose=" + targetPose);
                robot.robotDrive.purePursuitDrive.start(
                    owner, driveEvent, 2.0, true,
                    robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                    robot.robotInfo.profiledMaxDeceleration, targetPose);
                sm.addEvent(driveEvent);
                if (robot.elevatorArmTask != null)
                {
                    tracer.traceInfo(moduleName, "***** Moving Elevator and Arm to scoring position to reefLevel: " + taskParams.reefLevel);
                    robot.elevatorArmTask.setCoralScorePosition(owner, taskParams.reefLevel, elevatorArmEvent);
                    sm.addEvent(elevatorArmEvent);
                }
                sm.waitForEvents(State.DONE/*SCORE_CORAL*/, true, true);
                break;

            case SCORE_CORAL:
                // Code Review: Need to figure out the exact sequence to score the coral. Is there a reason why we won't have the coral?
                if (robot.coralGrabber.hasObject())
                {
                    robot.coralGrabber.autoEject(owner, 0.0, scoreEvent, 2.0);
                    sm.waitForSingleEvent(scoreEvent, State.DONE);
                }
                else
                {
                    tracer.traceInfo(owner, "Coral Grabber does not have Coral to score");
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
 
}   //class TaskAutoScoreCoral
