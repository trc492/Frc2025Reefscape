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
import teamcode.FrcAuto.ScorePickup;
import teamcode.FrcAuto.ScoreSide;
import teamcode.FrcAuto.StationSide;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAutoMiddle implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdAutoMiddle.class.getSimpleName();

    private enum State
    {
        START,
        SCORE_PRELOAD,
        DO_DELAY,
        GO_TO_CORAL_STATION,
        PICK_UP_CORAL,
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
    private StationSide stationSide;
    private ScoreSide scoreSide;
    private boolean relocalize;
    private ScorePickup scorePickup;
    private boolean scorePreload;
    private boolean goToStation;
    private boolean useAprilTagVision;


    private int coralScored;
    private int coralTarget;


    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAutoMiddle(Robot robot, AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAutoMiddle

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
                Score preloaded Coral in middle high branch
                Call Auto-Assist Score
                Delay, so then the alliance partner can score
                Drive to left or right coral station depending on choice
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
                case START:
                    // Set robot location according to auto choices.
                    // If this method does nothing then we will have to find another way to find our current position
                    robot.setRobotStartPosition(); // TODO: This method does nothing, do we need to add code to make it functional?
                    startPos = FrcAuto.autoChoices.getStartPos();
                    stationSide = FrcAuto.autoChoices.getStationSide();
                    scorePickup = FrcAuto.autoChoices.getScorePickup();
                    scoreSide = FrcAuto.autoChoices.getScoreSide();
                    relocalize = FrcAuto.autoChoices.getRelocalize();
                    goToStation = FrcAuto.autoChoices.goToStation();
                    scorePreload = FrcAuto.autoChoices.scorePreload();
                    useAprilTagVision = FrcAuto.autoChoices.useVision();
                    robot.globalTracer.traceInfo(moduleName, "****** Scoring preload from" + startPos + " at " + robot.robotDrive.driveBase.getFieldPosition());
                    // Depending on whether we want to score one coral from the station or two, it will set our coralTarget to one or two
                    if (scorePickup == FrcAuto.ScorePickup.SCORE_ONE)
                    {
                        coralTarget++; 
                    }
                    else if (scorePickup == FrcAuto.ScorePickup.SCORE_TWO)
                    {
                        coralTarget += 2;
                    }
                        
                    // Navigate to Reef position.
                    sm.setState(State.SCORE_PRELOAD);
                    break;

                case SCORE_PRELOAD:
                    // Score preloaded Coral to high branch.
                    if(scorePreload){
                        if(useAprilTagVision){
                            // Using vision to view AprilTag that is directly infront of us, and will AutoScore using this AprilTag
                            robot.globalTracer.traceInfo(moduleName, "***** Scoring preload using AprilTag Vision");
                            // TODO: Will have to add a dashboard input that will give scoreSide
                            robot.scoreCoralTask.autoScoreCoral(null, useAprilTagVision, 3, false, true, relocalize, scoreSide == ScoreSide.LEFT ? 0 : 1, event);
                            sm.waitForSingleEvent(event, goToStation ? State.DO_DELAY : State.DONE);
                        } else{
                            // // If we do not have vision, then we will go to this position using odometry
                            // robot.globalTracer.traceInfo(moduleName, "***** Scoring preload without AprilTag Vision");
                            // int coralAprilTagId = RobotParams.Game.APRILTAG_FAR_MID_REEF[alliance == Alliance.Red ? 0 : 1];
                            // TrcPose2D aprilTagCenterPose = RobotParams.Game.APRILTAG_POSES[coralAprilTagId - 1].clone();
                            // robot.robotDrive.purePursuitDrive.start(
                            //     event, 0.0, false, 
                            //     robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            //     robot.robotInfo.profiledMaxDeceleration, aprilTagCenterPose);
                            // // Calling AutoScore without Vision so it will just call the subsystem tasks
                            // // TODO: Will have to add a dashboard input that will give scoreSide
                            // robot.scoreCoralTask.autoScoreCoral(null, false, 3, false, true, relocalize, 0, event);
                            // sm.waitForSingleEvent(event, goToStation ? State.DO_DELAY : State.DONE);
                            robot.globalTracer.traceInfo(moduleName,"******* We do not have accurate Odometry to go to target");
                            sm.setState(State.DONE);
                        }
                        
                    } else{
                        // If our scorePreload is false, then we can go to DONE
                        robot.globalTracer.traceInfo(moduleName, "***** Not scoring preload, going to DONE");
                        sm.setState(State.DONE);
                    }
                    break;

                case DO_DELAY:
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.GO_TO_CORAL_STATION);
                    }
                    else
                    {
                        sm.setState(State.GO_TO_CORAL_STATION);
                    }
                    break;

                case GO_TO_CORAL_STATION:
                    if(stationSide == StationSide.PROCESSOR){
                        // Pure Pursuit to a position offset from AprilTags 8 or 16 depending on your alliance, so then we can call AutoPickupCoralFromStation to pick up the coral
                        robot.globalTracer.traceInfo(moduleName, "***** Driving to intermediate position for right coral station.");
                        int stationSideIntermediatePoseId = RobotParams.Game.APRILTAG_RIGHT_CORAL_STATION[alliance == Alliance.Red ? 0 : 1];
                        int middleReefAprilId              = RobotParams.Game.APRILTAG_FAR_MID_REEF[alliance == Alliance.Red ? 0 : 1];

                        TrcPose2D intermediatePose = RobotParams.Game.APRILTAG_POSES[middleReefAprilId - 1].clone();
                        intermediatePose.x += 85.0;

                        TrcPose2D stationSideAprilTagPose = RobotParams.Game.APRILTAG_POSES[stationSideIntermediatePoseId - 1].clone();
                        stationSideAprilTagPose.x -= 60;
                        stationSideAprilTagPose.y -= 40;

                        //stationSideIntermediatePose.angle = 60;
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, intermediatePose, stationSideAprilTagPose);
                        sm.waitForSingleEvent(event, State.PICK_UP_CORAL);


                    } else if(stationSide == StationSide.FAR){
                        // Same as above, but we are using Odometry to go to the intermediate position.
                        robot.globalTracer.traceInfo(moduleName, "***** Driving to intermediate position for left coral station.");
                        int stationSideIntermediatePoseId = RobotParams.Game.APRILTAG_LEFT_CORAL_STATION[alliance == Alliance.Red ? 0 : 1];
                        int middleReefAprilId              = RobotParams.Game.APRILTAG_FAR_MID_REEF[alliance == Alliance.Red ? 0 : 1];

                        TrcPose2D intermediatePose = RobotParams.Game.APRILTAG_POSES[middleReefAprilId - 1].clone();
                        intermediatePose.x -= 85.0;

                        TrcPose2D stationSideAprilTagPose = RobotParams.Game.APRILTAG_POSES[stationSideIntermediatePoseId - 1].clone();
                        stationSideAprilTagPose.x += 60;
                        stationSideAprilTagPose.y -= 40;
                        stationSideAprilTagPose.angle -= 180;

                        //stationSideIntermediatePose.angle = 60;
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, intermediatePose, stationSideAprilTagPose);
                        sm.waitForSingleEvent(event, State.PICK_UP_CORAL);
                    }
                                
                    break;
                case PICK_UP_CORAL:
                    // Pick up Coral from station.
                    if(useAprilTagVision){
                        // We are at the intermediate position, so now we can call AutoPickupCoralFromStation to pick up the coral
                        robot.globalTracer.traceInfo(moduleName, "***** Picking up coral from Station using AprilTag Vision");
                        robot.pickupCoralFromStationTask.autoPickupCoral(null, useAprilTagVision, true, relocalize, event);
                        sm.waitForSingleEvent(event, scorePickup == ScorePickup.SCORE_NONE ? State.DONE : State.SCORE_CORAL);
                    } else{
                        // Use odometry to go to the position of the AprilTag, and call AutoPickupCoralFromStation without vision to call the subsystem tasks
                        robot.globalTracer.traceInfo(moduleName, "***** Not using Vision");
                        int stationAprilTagId = stationSide == StationSide.FAR? RobotParams.Game.APRILTAG_LEFT_CORAL_STATION[alliance == Alliance.Red ? 0 : 1]: RobotParams.Game.APRILTAG_RIGHT_CORAL_STATION[alliance == Alliance.Red ? 0 : 1];
                        TrcPose2D aprilTagStationPose = RobotParams.Game.APRILTAG_POSES[stationAprilTagId - 1].clone();
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, aprilTagStationPose);
                        robot.pickupCoralFromStationTask.autoPickupCoral(null, false, true, relocalize, event);
                        sm.waitForSingleEvent(event, State.SCORE_CORAL);
                    }
                    break;

                case SCORE_CORAL:
                    if(useAprilTagVision){
                        // We are at the coral Station and the Camera can probably see the AprilTag, so we can call AutoScoreCoral to score the coral
                        robot.globalTracer.traceInfo(moduleName, "***** Going to score on Reef using AprilTag Vision");
                        // TODO: Will have to add a dashboard input that will give scoreSide    
                        robot.scoreCoralTask.autoScoreCoral(null, useAprilTagVision, 3, false, false, relocalize, scoreSide == ScoreSide.LEFT ? 0 : 1, event);
                        // Increase coral scored by one, if we reached the coralTarget, then we are done, otherwise we go back to GO_TO_CORAL_STATION
                        coralScored++;
                        sm.waitForSingleEvent(event, coralScored == coralTarget ? State.DONE : State.GO_TO_CORAL_STATION);
                    } else{
                        robot.globalTracer.traceInfo(moduleName, "***** Going to score on Reef without AprilTag Vision");
                        int pickupScoreId = stationSide == StationSide.FAR? RobotParams.Game.APRILTAG_CLOSE_RIGHT_REEF[alliance == Alliance.Red ? 0 : 1]: RobotParams.Game.APRILTAG_CLOSE_LEFT_REEF[alliance == Alliance.Red ? 0 : 1];
                        TrcPose2D pickupScorePose = RobotParams.Game.APRILTAG_POSES[pickupScoreId - 1].clone();
                        robot.robotDrive.purePursuitDrive.start(
                            event, 0.0, false,
                            robot.robotInfo.profiledMaxVelocity, robot.robotInfo.profiledMaxAcceleration,
                            robot.robotInfo.profiledMaxDeceleration, pickupScorePose);
                            // TODO: Will have to add a dashboard input that will give scoreSide
                        robot.scoreCoralTask.autoScoreCoral(null, false, 3, false, true, relocalize, scoreSide == ScoreSide.LEFT ? 0 : 1, event);
                        coralScored++;
                        sm.waitForSingleEvent(event, coralScored == coralTarget ? State.DONE : State.GO_TO_CORAL_STATION);
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

}   //class CmdAutoMiddle
