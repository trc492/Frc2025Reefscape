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

package teamcode.autocommands;

import teamcode.FrcAuto;
import teamcode.Robot;
import teamcode.RobotParams;
import teamcode.tasks.TaskAutoScoreCoral.ScoreCoralOffset;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frclib.vision.FrcPhotonVision;
import teamcode.FrcAuto.AutoChoices;
import teamcode.FrcAuto.AutoStartPos;
import teamcode.FrcAuto.StationSide;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdReefscapeAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdReefscapeAuto.class.getSimpleName();

    private enum State
    {
        START,
        SCORE_PRELOAD,
        GO_TO_CORAL_STATION,
        PICKUP_CORAL,
        SCORE_CORAL,
        APPROACH_REEF,
        DONE
    }   //enum State

    private final Robot robot;
    private final AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private FrcAuto.AutoStartPos startPos;
    private Alliance alliance;
    private boolean useVision;
    private boolean relocalize;
    private boolean goToStation;
    private StationSide stationSide;
    private int stationPickupCount;
    private double visionXOffset;
    private double visionYOffset;

    private int reefAprilTagId = -1;
    private int stationAprilTagId = -1;
    private boolean scoreRightSide = true;
    private int preloadAprilTagId;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdReefscapeAuto(Robot robot, AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdReefscapeAuto

    //
    // Implements the TrcRobot.RobotCommand interface.
    //

    /**
     * This method checks if the current RobotCommand  is running.
     *
     * @return true if the command is running, false otherwise.
     */
    @Override
    public boolean isActive()
    {
        return sm.isEnabled();
    }   //isActive

    /**
     * This method cancels the command if it is active.
     */
    @Override
    public void cancel()
    {
        timer.cancel();
        sm.stop();
    }   //cancel

    /**
     * This method must be called periodically by the caller to drive the command sequence forward.
     *
     * @param elapsedTime specifies the elapsed time in seconds since the start of the robot mode.
     * @return true if the command sequence is completed, false otherwise.
     */
    @Override
    public boolean cmdPeriodic(double elapsedTime)
    {
        State state = sm.checkReadyAndGetState();

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition();
                    // Retrieve auto choice options.
                    startPos = autoChoices.getStartPos();
                    alliance = autoChoices.getAlliance();
                    useVision = autoChoices.useVision();
                    relocalize = autoChoices.getRelocalize();
                    goToStation = autoChoices.goToStation();
                    stationSide = autoChoices.getStationSide();
                    stationPickupCount = autoChoices.getStationPickupCount();
                    visionXOffset = autoChoices.getVisionXOffset();
                    visionYOffset = autoChoices.getVisionYOffset();
                    // Do delay if necessary.
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.SCORE_PRELOAD);
                    }
                    else
                    {
                        sm.setState(State.SCORE_PRELOAD);
                    }
                    break;

                case SCORE_PRELOAD:
                    if (autoChoices.scorePreload())
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Score Preload.");
                        // Determine the AprilTagId to look for.
                        if (startPos == AutoStartPos.START_POSE_PROCESSOR)
                        {
                            preloadAprilTagId =
                                RobotParams.Game.APRILTAG_FAR_RIGHT_REEF[alliance == Alliance.Red? 0: 1];
                            robot.scoreCoralTask.autoScoreCoral(
                                null, useVision, preloadAprilTagId, 3, true, false, relocalize, false, 0.3,
                                new ScoreCoralOffset(
                                    visionXOffset + (scoreRightSide? 5.0: -10.5), visionYOffset - 16.5),
                                event);
                        }
                        else if (startPos == AutoStartPos.START_POSE_FAR_SIDE)
                        {
                            preloadAprilTagId =
                                RobotParams.Game.APRILTAG_FAR_LEFT_REEF[alliance == Alliance.Red? 0: 1];
                            robot.scoreCoralTask.autoScoreCoral(
                                null, useVision, preloadAprilTagId, 3, true, false, relocalize, false, 0.3,
                                new ScoreCoralOffset(
                                    visionXOffset + (scoreRightSide? 8.5: -10.5), visionYOffset - 22.5),
                                event);
                        }
                        else
                        {
                            preloadAprilTagId =
                                RobotParams.Game.APRILTAG_FAR_MID_REEF[alliance == Alliance.Red? 0: 1];
                            robot.scoreCoralTask.autoScoreCoral(
                                null, useVision, preloadAprilTagId, 3, true, false, relocalize, false, 0.2,
                                new ScoreCoralOffset(
                                    visionXOffset + (scoreRightSide? 8.5: -10.5), visionYOffset - 18.5),
                                event);
                        }
                        sm.waitForSingleEvent(event, State.GO_TO_CORAL_STATION);
                    }
                    else
                    {
                        sm.setState(State.GO_TO_CORAL_STATION);
                    }
                    break;

                case GO_TO_CORAL_STATION:
                    robot.turtle();
                    if (goToStation)
                    {
                        TrcPose2D intermediatePose = null;
                        // If we haven't already, determine the Coral Station AprilTag ID to look for.
                        // If we are fetching the 2nd Coral from the Station, we already determined the AprilTag ID
                        // last time.
                        if (stationAprilTagId == -1)
                        {
                            if (startPos == AutoStartPos.START_POSE_PROCESSOR)
                            {
                                stationAprilTagId =
                                    RobotParams.Game.APRILTAG_RIGHT_CORAL_STATION[alliance == Alliance.Red? 0: 1];
                                intermediatePose = RobotParams.Game.PROCESSOR_SIDE_LOOKOUT_BLUE;
                            }
                            else if (startPos == AutoStartPos.START_POSE_FAR_SIDE)
                            {
                                stationAprilTagId =
                                    RobotParams.Game.APRILTAG_LEFT_CORAL_STATION[alliance == Alliance.Red? 0: 1];
                                intermediatePose = RobotParams.Game.FAR_SIDE_LOOKOUT_BLUE;
                            }
                            else
                            {
                                stationAprilTagId =
                                    RobotParams.Game.APRILTAG_STATION[stationSide == StationSide.PROCESSOR? 0: 1]
                                                                     [alliance == Alliance.Red? 0: 1];
                                intermediatePose = (stationSide == StationSide.PROCESSOR?
                                    RobotParams.Game.PROCESSOR_SIDE_LOOKOUT_BLUE:
                                    RobotParams.Game.FAR_SIDE_LOOKOUT_BLUE).clone();
                                // Center start position needs to have an intermediate point further down to avoid the
                                // reef.
                                intermediatePose.y += 108.0;
                            }
                            intermediatePose = robot.adjustPoseByAlliance(intermediatePose, alliance);
                        }

                        TrcPose2D aprilTagPose = FrcPhotonVision.getAprilTagFieldPose(stationAprilTagId);
                        TrcPose2D targetPose = robot.adjustPoseByOffset(aprilTagPose, 20.0, -45.0);
                        // AprilTag angle is reversed from the robot.
                        targetPose.angle -= 180.0;
                        if (intermediatePose != null &&
                            Math.abs(targetPose.y - intermediatePose.y) > RobotParams.Field.LENGTH)
                        {
                            // Trying to catch the bug where the calculation of intermediatePose somehow landed
                            // a point to the opposite side of the field.
                            robot.globalTracer.traceWarn(
                                moduleName,
                                "\n\tAlliance=" + alliance +
                                "\n\tStartPos=" + startPos +
                                "\n\tStationSide=" + stationSide +
                                "\n\tStationAprilTagId=" + stationAprilTagId +
                                "\n\tIntermediatePose=" + intermediatePose +
                                "\n\tTargetPose=" + targetPose);
                        }
                        robot.globalTracer.traceInfo(
                            moduleName,
                            "***** Go to Coral Station: AprilTag=" + stationAprilTagId +
                            ", AprilTagPose=" + aprilTagPose +
                            ", IntermediatePose=" + intermediatePose +
                            ", TargetPose=" + targetPose);

                        if (intermediatePose != null)
                        {
                            // We are coming from scoring preload, so we need to have intermediate point to avoid the
                            // Reef.
                            robot.robotDrive.purePursuitDrive.start(
                                null, event, 0.0, false, robot.robotInfo.profiledMaxVelocity,
                                robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                                intermediatePose, targetPose);
                                sm.waitForSingleEvent(event, State.PICKUP_CORAL);
                        }
                        else
                        {
                            // We are coming from the scoring a Station Coral, so we have a straight shot back to the
                            // station.
                            robot.robotDrive.purePursuitDrive.start(
                                null, event, 0.0, false, robot.robotInfo.profiledMaxVelocity,
                                robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                                targetPose);
                                sm.waitForSingleEvent(event, State.PICKUP_CORAL);
                        }
                    }
                    else
                    {
                        // Not going to the coral station, we are done.
                        sm.setState(State.DONE);
                    }
                    break;

                case PICKUP_CORAL:
                    // Check if we need to pick up a Coral from the Station.
                    if (stationPickupCount > 0)
                    {
                        robot.pickupCoralFromStationTask.autoPickupCoral(
                            null, useVision, stationAprilTagId, relocalize, event);
                        sm.waitForSingleEvent(event, State.APPROACH_REEF);
                    }
                    else
                    {
                        // No more Coral to pick up, we are done.
                        sm.setState(State.DONE);
                    }
                    break;

                case APPROACH_REEF:
                    // If we haven't already, determine the Reef AprilTag ID to look for.
                    // If we are scoring the 2nd Coral from the Station, we already determined the AprilTag ID
                    // last time.
                    if (reefAprilTagId == -1)
                    {
                        if (startPos == AutoStartPos.START_POSE_PROCESSOR)
                        {
                            reefAprilTagId =
                                RobotParams.Game.APRILTAG_CLOSE_RIGHT_REEF[alliance == Alliance.Red? 0: 1];
                        }
                        else if (startPos == AutoStartPos.START_POSE_FAR_SIDE)
                        {
                            reefAprilTagId =
                                RobotParams.Game.APRILTAG_CLOSE_LEFT_REEF[alliance == Alliance.Red? 0: 1];
                        }
                        else
                        {
                            if (stationSide == StationSide.PROCESSOR)
                            {
                                reefAprilTagId =
                                    RobotParams.Game.APRILTAG_CLOSE_RIGHT_REEF[alliance == Alliance.Red? 0: 1];
                            }
                            else
                            {
                                reefAprilTagId =
                                    RobotParams.Game.APRILTAG_CLOSE_LEFT_REEF[alliance == Alliance.Red? 0: 1];
                            }
                        }
                    }
                    // Fire and forget assuming PurePursuit and vision auto score is slower.
                    robot.elevatorArmTask.setCoralScorePosition(null, 3, null);
                    TrcPose2D reefAprilTagPose = FrcPhotonVision.getAprilTagFieldPose(reefAprilTagId);
                    TrcPose2D reefTargetPose = robot.adjustPoseByOffset(reefAprilTagPose, 5.0, -55.0);
                    robot.robotDrive.purePursuitDrive.start(
                        null, event, 0.0, false, robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration, robot.robotInfo.profiledMaxDeceleration,
                        reefTargetPose);
                    sm.waitForSingleEvent(event, State.SCORE_CORAL);
                    break;

                case SCORE_CORAL:
                    robot.scoreCoralTask.autoScoreCoral(
                        null, useVision, reefAprilTagId, 3, scoreRightSide, false, relocalize, false, 0.2,
                        new ScoreCoralOffset(scoreRightSide? 6.5: -10.5, -19.5), event);
                    // Decrement the number of station pickup and flip to the other side.
                    stationPickupCount--;
                    scoreRightSide = !scoreRightSide;
                    sm.waitForSingleEvent(event, stationPickupCount > 0? State.GO_TO_CORAL_STATION: State.DONE);
                    break;

                default:
                case DONE:
                    // We are done.
                    cancel();
                    break;
            }
            robot.globalTracer.tracePostStateInfo(
                sm.toString(), state, robot.robotDrive.driveBase, robot.robotDrive.pidDrive,
                robot.robotDrive.purePursuitDrive, null);
        }

        return !sm.isEnabled();
    }   //cmdPeriodic

}   //class CmdReefscapeAuto
