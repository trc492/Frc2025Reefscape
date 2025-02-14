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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import teamcode.FrcAuto.AutoChoices;
import teamcode.FrcAuto.AutoStartPos;
import teamcode.FrcAuto.AutoStrategy;
import teamcode.FrcAuto.ScorePickup;
import teamcode.FrcAuto.StationSide;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAutoSide implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdAutoSide.class.getSimpleName();

    private enum State
    {
        START,
        SCORE_PRELOAD,
        DO_DELAY,
        GO_TO_CORAL_STATION,
        PICKUP_CORAL,
        GO_TO_REEF,
        SCORE_CORAL,
        DONE
    }   //enum State

    private final Robot robot;
    private final AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcStateMachine<State> sm;

    private Alliance alliance;
    private AutoStartPos startPos;
    private AutoStrategy autoStrategy;
    private StationSide stationSide;
    private ScorePickup scorePickup;
    private double startDelay;
    private boolean relocalize;
    private boolean goToStation;
    private boolean scorePreload;

    private int coralScored;
    private int coralTarget;

    private String cyclePositions;
    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAutoSide(Robot robot, AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAutoSide

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
            /*
                Score preloaded Coral in side high branch
                Call Auto-Assist Score
                Delay, so then the alliance partner can score
                Drive to coral station
                PurePursuit out to Abs position to see station AprilTag
                Pickup 1st coral from coral station
                Use Auto-Assist Pickup From Station from Abs pos
                Score 1st coral pickup to high branch
                PurePursuit from station to another Abs pos in front of Coral
                Use Auto-Assist Score to score
                Pickup 2nd coral from coral station
                Repeat same as above
                Score 2nd coral pickup to high branch
                Approach Station(Attempt to pickup another, but we will most likely get cutoff)
             */
            switch (state)
            {
                // NOTE: Daniel has not made auto choices yet so I am putting in values in for now 
                // + Sarah hasn't finish auto pickup task so putting in values for that as well
                case START:
                    // Set robot location according to auto choices.
                    robot.setRobotStartPosition(autoChoices);
                    // Initialize auto choices.
                    alliance = FrcAuto.autoChoices.getAlliance();
                    autoStrategy = FrcAuto.autoChoices.getStrategy();
                    startPos = FrcAuto.autoChoices.getStartPos();
                    stationSide = FrcAuto.autoChoices.getStationSide();
                    scorePickup = FrcAuto.autoChoices.getScorePickup();
                    startDelay = FrcAuto.autoChoices.getStartDelay();
                    relocalize = FrcAuto.autoChoices.getRelocalize();
                    goToStation = FrcAuto.autoChoices.goToStation();
                    scorePreload = FrcAuto.autoChoices.scorePreload();
                    // Set score variables
                    if (scorePickup == FrcAuto.ScorePickup.SCORE_ONE)
                    {
                        coralTarget++; // increase coral target by 1
                    }
                    else if (scorePickup == FrcAuto.ScorePickup.SCORE_TWO)
                    {
                        coralTarget += 2; // increase coral target by 2
                    }
                    // Set cycle positions
                    cyclePositions = "Side";
                    // Navigate to Reef position.
                    if (scorePreload)
                    {
                        coralTarget++; // increase coral target by 1 to include preload
                        TrcPose2D scorePreloadPos = startPos == FrcAuto.AutoStartPos.START_POSE_PROCESSOR ? 
                            RobotParams.Game.PROCESSOR_SCORE_CORAL_START_RED : RobotParams.Game.FAR_SCORE_CORAL_START_RED;
                        robot.robotDrive.purePursuitDrive.start(
                        event, 0.0, false,
                        robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                        robot.robotInfo.profiledMaxDeceleration, scorePreloadPos);
                        sm.waitForSingleEvent(event, State.SCORE_PRELOAD);
                    }
                    else
                    {
                        sm.setState(State.DO_DELAY);
                    }
                    break;

                case SCORE_PRELOAD:
                    // Score preloaded Coral to high branch.
                    robot.scoreCoralTask.autoScoreCoral(RobotParams.Preferences.useVision, 3, false, true, relocalize, true, event);
                    coralScored++;
                    if (coralScored < coralTarget)
                    {
                        sm.waitForSingleEvent(event, State.DO_DELAY);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
                    break;

                case DO_DELAY:
                    if (startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        timer.set(startDelay, event);
                        if (goToStation)
                        {
                            sm.waitForSingleEvent(event, State.GO_TO_CORAL_STATION);
                        }
                        else
                        {
                            sm.setState(State.DONE);
                        }
                    }
                    else
                    {
                        if (goToStation)
                        {
                            sm.waitForSingleEvent(event, State.GO_TO_CORAL_STATION);
                        }
                        else
                        {
                            sm.setState(State.DONE);
                        }
                    }
                    break;

                case GO_TO_CORAL_STATION:
                    // Navigate to Coral Station.
                    if (cyclePositions.equals("Side"))
                    {
                        TrcPose2D stationSidePos = startPos == FrcAuto.AutoStartPos.START_POSE_PROCESSOR ? 
                            RobotParams.Game.PROCESSOR_PICKUP_CORAL_SIDE_RED : RobotParams.Game.FAR_PICKUP_CORAL_SIDE_RED;
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, stationSidePos);
                        sm.waitForSingleEvent(event, State.PICKUP_CORAL);
                    }
                    else if (cyclePositions.equals("Center"))
                    {
                        TrcPose2D stationCenterPos = startPos == FrcAuto.AutoStartPos.START_POSE_PROCESSOR ? 
                            RobotParams.Game.PROCESSOR_PICKUP_CORAL_CENTER_RED : RobotParams.Game.FAR_PICKUP_CORAL_CENTER_RED;
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, stationCenterPos);
                        sm.waitForSingleEvent(event, State.PICKUP_CORAL); 
                    }
                    break;

                case PICKUP_CORAL:
                    // TODO: adjust once Sarah finishes the auto task
                    // Pick up Coral from station.
                    robot.pickupCoralFromStationTask.autoPickupCoral(RobotParams.Preferences.useVision, true, relocalize, event);
                    sm.waitForSingleEvent(event, State.GO_TO_REEF);
                    break;

                case GO_TO_REEF:
                    // Navigate to Reef.
                    if (cyclePositions == "Side")
                    {
                        TrcPose2D scoreSidePos = startPos == FrcAuto.AutoStartPos.START_POSE_PROCESSOR ? 
                            RobotParams.Game.PROCESSOR_SCORE_CORAL_SIDE_RED : RobotParams.Game.FAR_SCORE_CORAL_SIDE_RED;
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, scoreSidePos);
                        sm.waitForSingleEvent(event, State.SCORE_CORAL); 
                    }
                    else if (cyclePositions == "Center")
                    {
                        TrcPose2D stationCenterPos = startPos == FrcAuto.AutoStartPos.START_POSE_PROCESSOR ? 
                            RobotParams.Game.PROCESSOR_PICKUP_CORAL_CENTER_RED : RobotParams.Game.FAR_PICKUP_CORAL_CENTER_RED;
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, stationCenterPos);
                        sm.waitForSingleEvent(event, State.SCORE_CORAL); 
                    }
                    break;

                case SCORE_CORAL:
                    // Score Coral to high branch.
                    robot.scoreCoralTask.autoScoreCoral(RobotParams.Preferences.useVision, 3, false, true, relocalize, true, event);
                    coralScored++;
                    cyclePositions = "Center";
                    if (coralScored < coralTarget)
                    {
                        sm.waitForSingleEvent(event, State.GO_TO_CORAL_STATION);
                    }
                    else
                    {
                        sm.setState(State.DONE);
                    }
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

}   //class CmdAutoSide
