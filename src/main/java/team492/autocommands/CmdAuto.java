/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
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

package team492.autocommands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frclib.vision.FrcPhotonVision.DetectedObject;
import team492.FrcAuto;
import team492.Robot;
import team492.RobotParams;
import team492.FrcAuto.AutoChoices;
import team492.FrcAuto.AutoStartPos;
import team492.FrcAuto.EndAction;
import team492.FrcAuto.ScoreWingNotes;
import team492.autotasks.TaskAutoScoreNote.TargetType;
import trclib.pathdrive.TrcPose2D;
import trclib.pathdrive.TrcWaypoint;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcStateMachine;
import trclib.timer.TrcTimer;

/**
 * This class implements an autonomous strategy.
 */
public class CmdAuto implements TrcRobot.RobotCommand
{
    private static final String moduleName = CmdAuto.class.getSimpleName();

    private enum State
    {
        START,
        DO_DELAY,
        DRIVE_TO_WING_NOTE,
        PICKUP_WING_NOTE,
        SCORE_NOTE_TO_AMP,
        CLEAR_THE_POST,
        TURN_TO_SPEAKER,
        TURN_TO_CENTERLINE,
        SCORE_NOTE_TO_SPEAKER,
        TURN_TO_WING_NOTES,
        DRIVE_TO_CENTERLINE,
        PERFORM_END_ACTION,
        PICKUP_CENTERLINE_NOTE,
        DRIVE_TO_SPEAKER,
        DRIVE_TO_AMP,
        PARK,
        DONE
    }   //enum State

    private final Robot robot;
    private final AutoChoices autoChoices;
    private final TrcTimer timer;
    private final TrcEvent event;
    private final TrcEvent aprilTagEvent;
    private final TrcEvent noteEvent;
    private final TrcStateMachine<State> sm;
    private Alliance alliance;
    private AutoStartPos startPos;
    private ScoreWingNotes scoreWingNotes;
    private EndAction endAction;
    private boolean relocalize;
    private int numWingNotesScored = 0;
    private int numCenterlineNotesScored = 0;
    private boolean aprilTagVisionEnabled = false;
    private boolean visionGuidanceEnabled = false;
    private boolean noteVisionEnabled = false;
    private double noteDistanceThreshold = RobotParams.Intake.noteDistanceThreshold;
    private double noteAngleThreshold = RobotParams.Intake.noteAngleThreshold;
    private boolean performingEndAction = false;
    private int centerlineIndex = 0;

    // private Double visionExpiredTime = null;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object for providing access to various global objects.
     * @param autoChoices specifies the autoChoices object.
     */
    public CmdAuto(Robot robot, AutoChoices autoChoices)
    {
        this.robot = robot;
        this.autoChoices = autoChoices;

        timer = new TrcTimer(moduleName);
        event = new TrcEvent(moduleName);
        aprilTagEvent = new TrcEvent(moduleName + ".aprilTagEvent");
        noteEvent = new TrcEvent(moduleName + ".noteEvent");
        sm = new TrcStateMachine<>(moduleName);
        sm.start(State.START);
    }   //CmdAuto

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

        // Must have vision to perform this autonomous.


        if (aprilTagVisionEnabled)
        {
            // Use vision to relocalize robot's position.
            DetectedObject aprilTagObj;
            if (startPos != AutoStartPos.AMP)
            {
                 aprilTagObj = robot.photonVisionFront.getBestDetectedAprilTag(aprilTagEvent, 4, 7, 3, 8);
            }
            else
            {
                aprilTagObj = robot.photonVisionFront.getBestDetectedAprilTag(aprilTagEvent, 4, 7, 5, 6, 3, 8);
            }

            if (visionGuidanceEnabled)
            {
                if (aprilTagObj != null)
                {
                    robot.relocalize(aprilTagObj);
                }
                // else
                // {
                //     robot.globalTracer.traceInfo(moduleName, "***** Vision Guidance: AprilTag not found.");
                // }
            }
        }

        if (noteVisionEnabled)
        {
            DetectedObject noteObj = robot.photonVisionBack.getBestDetectedObject();

            if (noteObj != null)
            {
                TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                if (noteObj.targetPose.y > noteDistanceThreshold ||
                    Math.abs(noteObj.targetPose.angle) > noteAngleThreshold)
                {
                    robot.globalTracer.traceInfo(
                        moduleName,
                        "***** Vision found note too far or not turn enough: notePose=" + noteObj.targetPose +
                        ", robotPose=" + robotPose);
                }
                else
                {
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Vision found note: notePose=" + noteObj.targetPose +
                        ", robotPose=" + robotPose);
                    noteEvent.signal();
                }
            }
        }

        if (state == null)
        {
            robot.dashboard.displayPrintf(8, "State: disabled or waiting (nextState=" + sm.getNextState() + ")...");
        }
        else
        {
            TrcPose2D robotPose;
            TrcPose2D targetPose;
            TrcPose2D intermediatePose;
            TrcPose2D intermediatePose2;
            TrcPose2D intermediatePose3;

            robot.dashboard.displayPrintf(8, "State: " + state);
            robot.globalTracer.tracePreStateInfo(sm.toString(), state);
            switch (state)
            {
                case START:
                    // Set robot start field position and score preloaded Note.
                    robot.robotDrive.setFieldPosition(false);
                    alliance = FrcAuto.autoChoices.getAlliance();
                    startPos = FrcAuto.autoChoices.getStartPos();
                    scoreWingNotes = FrcAuto.autoChoices.getScoreWingNotes();
                    endAction = FrcAuto.autoChoices.getEndAction();
                    relocalize = FrcAuto.autoChoices.getRelocalize();
                    robot.shooter.tracer.setTraceLevel(TrcDbgTrace.MsgLevel.DEBUG);
                    // Shoot pre-load.
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Shoot Preload from " + startPos +
                        " at " + robot.robotDrive.driveBase.getFieldPosition() + ".");
                    if (startPos == AutoStartPos.AMP)
                    {
                        robot.autoScoreNote.autoAssistScore(TargetType.Amp, false, true, relocalize, event);
                    }
                    else
                    {
                        // Don't turn off shooter after shooting.
                        robot.shooter.aimShooter(
                            null, RobotParams.Shooter.shooterSpeakerCloseVelocity,
                            RobotParams.Shooter.tiltSpeakerCloseAngle, 0.0, event, 0.0, robot::shoot, null);
                    }
                    sm.waitForSingleEvent(event, State.DO_DELAY);
                    break;

                case DO_DELAY:
                    if (startPos != AutoStartPos.AMP)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Set shooter to Wing Note preset.");
                        robot.shooter.aimShooter(
                            RobotParams.Shooter.wingNotePresetParams.shooterVelocity,
                            RobotParams.Shooter.wingNotePresetParams.tiltAngle, 0.0);
                    }
                    // Do delay if there is one.
                    double startDelay = autoChoices.getStartDelay();
                    if (startDelay > 0.0)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Do delay " + startDelay + "s.");
                        timer.set(startDelay, event);
                        sm.waitForSingleEvent(event, State.DRIVE_TO_WING_NOTE);
                    }
                    else
                    {
                        sm.setState(State.DRIVE_TO_WING_NOTE);
                    }
                    break;

                case DRIVE_TO_WING_NOTE:
                    if (scoreWingNotes == ScoreWingNotes.SCORE_NONE)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** No Scoring Wing Note.");
                        sm.setState(State.DRIVE_TO_CENTERLINE);
                    }
                    else
                    {
                        // For SW_AMP_SIDE or SW_SOURCE_SIDE, we need to get to the position where the camera can
                        // see the Note.
                        if(startPos == AutoStartPos.SW_SOURCE_SIDE || startPos == AutoStartPos.SW_AMP_SIDE)
                        {
                            robot.globalTracer.traceInfo(
                                moduleName,
                                "***** Drive to position so camera can see Wing Note, turn on Note Vision.");
                            robotPose = robot.robotDrive.driveBase.getFieldPosition();
                            targetPose =
                                RobotParams.Game.wingNotePoses[0][startPos == AutoStartPos.SW_SOURCE_SIDE? 0: 2]
                                .clone();
                            targetPose.angle = 180.0;
                            intermediatePose = targetPose.clone();
                            intermediatePose.y -= 36.0;
                            targetPose.y -= 12.0;
                            
                            robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                robot.adjustPoseByAlliance(intermediatePose, alliance),
                                robot.adjustPoseByAlliance(targetPose, alliance));
                            sm.addEvent(event);
                            enableNoteVision();
                            sm.addEvent(noteEvent);
                            sm.waitForEvents(State.PICKUP_WING_NOTE, false);
                        }
                        else if (startPos == AutoStartPos.AMP)
                        {
                            robot.globalTracer.traceInfo(
                                moduleName,
                                "***** Turn away from Amp so camera can see Wing Note, turn on Note Vision.");
                            robotPose = robot.robotDrive.driveBase.getFieldPosition();
                            intermediatePose = robotPose.clone();
                            intermediatePose.x += 15.0;
                            intermediatePose.angle = alliance == Alliance.Red? -30.0: -150.0;
                            targetPose = intermediatePose.clone();
                            targetPose.x += 15.0;
                            targetPose.angle = alliance == Alliance.Red? 0: 180.0;
                            // Turn on Note Vision at intermediatePose so that it will see the first Wing note instead
                            // of the middle one.
                            robot.robotDrive.purePursuitDrive.setWaypointEventHandler(this::waypointHandler);
                            robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                intermediatePose, targetPose);
                            sm.addEvent(event);
                            sm.waitForEvents(State.PICKUP_WING_NOTE, false);
                        }
                        else
                        {
                            sm.setState(State.PICKUP_WING_NOTE);
                        }
                    }
                    break;

                case PICKUP_WING_NOTE:
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Pickup Wing Note " + (numWingNotesScored + 1) +
                        ": Cancel PurePursuit, turn off Note Vision.");
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.robotDrive.purePursuitDrive.setWaypointEventHandler(null);
                    disableNoteVision();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.autoPickupFromGround.autoAssistPickup(true, true, event);
                    sm.waitForSingleEvent(
                        event,
                        startPos == AutoStartPos.AMP && numWingNotesScored == 0? State.SCORE_NOTE_TO_AMP:
                        startPos == AutoStartPos.SW_SOURCE_SIDE && numWingNotesScored == 0?
                            State.CLEAR_THE_POST: State.TURN_TO_SPEAKER);
                    break;

                case SCORE_NOTE_TO_AMP:
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Scoring Wing Note " + (numWingNotesScored + 1) +
                        " or Centerline Note " + (numCenterlineNotesScored + 1) + " into the Amp.");
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotDrive.purePursuitDrive.setWaypointEventHandler(null);
                    disableAprilTagVision();
                    robot.autoScoreNote.autoAssistScore(TargetType.Amp, true, true, relocalize, event);
                    if (performingEndAction)
                    {
                        numCenterlineNotesScored++;
                    }
                    else
                    {
                        numWingNotesScored++;
                    }
                    sm.waitForSingleEvent(
                        event, performingEndAction? State.DRIVE_TO_CENTERLINE: State.TURN_TO_WING_NOTES);
                    break;

                case CLEAR_THE_POST:
                    // Scoring the first Wing Note from the Source side. This state is only for scoring first Wing
                    // Note from starting pos of SW_SOURCE_SIDE. If we are scoring the third Wing Note from the
                    // SW_AMP_SIDE, the robot is not facing the Speaker so CLEAR_THE_POST won't work because it is
                    // moving robot forward relatively.
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Too close to stage post, move a bit forward to clear it.");
                    robot.robotDrive.purePursuitDrive.start(
                        event, 0.5, robot.robotDrive.driveBase.getFieldPosition(), true,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                        new TrcPose2D(0.0, 24.0, 0.0));
                    sm.waitForSingleEvent(event, State.TURN_TO_SPEAKER);
                    break;

                case TURN_TO_SPEAKER:
                    if (!robot.intake.hasObject())
                    {
                        // Failed to pick up a Note, probably vision failed, go to EndAction.
                        robot.globalTracer.traceInfo(moduleName, "***** Failed to pick up a Note, go to EndAction.");
                        sm.setState(State.TURN_TO_CENTERLINE);
                    }
                    else if (numWingNotesScored > 0)
                    {
                        // We are scoring the 2nd or 3rd Wing Note to the Speaker, so must turn towards the Speaker.
                        robot.globalTracer.traceInfo(
                            moduleName, "***** Turn to Speaker to score: Wing Note " + (numWingNotesScored + 1) +
                            " or Centerline Note " + (numCenterlineNotesScored + 1) + ".");
                        robotPose = robot.robotDrive.driveBase.getFieldPosition();
                        targetPose = robotPose.clone();
                        if (Math.abs(robotPose.x - RobotParams.Game.WINGNOTE_BLUE_SOURCE_SIDE.x) <
                            RobotParams.Game.PROXIMITY_THRESHOLD)
                        {
                            // Scoring SW_SOURCE_SIDE Wing Note.
                            targetPose.angle = alliance == Alliance.Red? -30.0: 210.0;
                        }
                        else if (Math.abs(robotPose.x - RobotParams.Game.WINGNOTE_BLUE_AMP_SIDE.x) <
                                 RobotParams.Game.PROXIMITY_THRESHOLD)
                        {
                            // Scoring SW_AMP_SIDE Wing Note.
                            targetPose.angle = alliance == Alliance.Red? 30.0: 150.0;
                        }
                        else
                        {
                            // Scoring SW_CENTER Wing Note.
                            targetPose.angle = alliance == Alliance.Red? 0.0: 180.0;
                        }

                        if (Math.abs(robotPose.x - RobotParams.Game.WINGNOTE_BLUE_SOURCE_SIDE.x) <
                            RobotParams.Game.PROXIMITY_THRESHOLD)
                        {
                            // Scoring the third Wing Note from the Amp side, need to avoid the stage post.
                            targetPose.y += alliance == Alliance.Red? 12.0: -12.0;
                            robot.globalTracer.traceInfo(
                                moduleName, "***** Scoring 2nd or 3rd Wing Note at " + targetPose +
                                " but move a bit forward to avoid the stage post.");
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, "***** Scoring 2nd or 3rd Wing Note at " + targetPose + ".");
                        }

                        robot.robotDrive.purePursuitDrive.start(
                            event, robotPose, false,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                            targetPose);
                        sm.addEvent(event);
                        enableAprilTagVision(performingEndAction);
                        sm.addEvent(aprilTagEvent);

                        sm.waitForEvents(State.SCORE_NOTE_TO_SPEAKER, false);
                    }
                    else
                    {
                        sm.setState(State.SCORE_NOTE_TO_SPEAKER);
                    }
                    break;

                case TURN_TO_CENTERLINE:
                    // We missed picking up a Note, go to EndAction but we need to turn to the Centerline first.
                    robot.globalTracer.traceInfo(
                        moduleName, "***** We missed picking up a Note, turn towards centerline to look for Note.");
                    robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robot.robotDrive.purePursuitDrive.start(
                        event, robotPose, false,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                        new TrcPose2D(robotPose.x, robotPose.y, alliance == Alliance.Red? 0.0: 180.0));
                    sm.waitForSingleEvent(event, State.DRIVE_TO_CENTERLINE);
                    break;

                case SCORE_NOTE_TO_SPEAKER:
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(1.0);
                    robot.robotDrive.purePursuitDrive.disableFixedHeading();
                    robot.disableAprilTagTracking();
                    disableAprilTagVision();
                    robot.autoScoreNote.autoAssistScore(TargetType.Speaker, true, true, relocalize, event);
                    if (performingEndAction)
                    {
                        numCenterlineNotesScored++;
                    }
                    else
                    {
                        numWingNotesScored++;
                    }
                    robot.globalTracer.traceInfo(
                        moduleName, "***** AutoScore Wing Note " + numWingNotesScored +
                        " or Centerline Note " + numCenterlineNotesScored + " to Speaker.");
                    sm.waitForSingleEvent(
                        event, performingEndAction? State.DRIVE_TO_CENTERLINE: State.TURN_TO_WING_NOTES);
                    break;

                case TURN_TO_WING_NOTES:
                    // If we start at SW_CENTER, we don't do 2nd and 3rd Wing Note.
                    if (startPos != AutoStartPos.SW_CENTER &&
                        (scoreWingNotes == ScoreWingNotes.SCORE_TWO && numWingNotesScored < 2 ||
                         scoreWingNotes == ScoreWingNotes.SCORE_THREE && numWingNotesScored < 3))
                    {
                        if (startPos == AutoStartPos.AMP && numWingNotesScored == 1)
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, "***** Finished scoring 1st Wing Note to Amp, already facing Wing Note.");
                            sm.setState(State.PICKUP_WING_NOTE);
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, "***** Turn to face Wing Note.");
                            robotPose = robot.robotDrive.driveBase.getFieldPosition();
                            targetPose = robotPose.clone();
                            targetPose.angle =
                                startPos == AutoStartPos.SW_SOURCE_SIDE? 90.0: -90.0;
                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                targetPose);
                            sm.addEvent(event);
                            enableNoteVision();
                            sm.addEvent(noteEvent);
                            sm.waitForEvents(State.PICKUP_WING_NOTE, false);
                        }
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** Done scoring Wing Notes.");
                        sm.setState(State.DRIVE_TO_CENTERLINE);
                    }
                    break;

                case DRIVE_TO_CENTERLINE:
                    if (endAction == EndAction.JUST_STOP ||
                        endAction == EndAction.SCORE_ONE_NOTE && numCenterlineNotesScored == 1 ||
                        endAction == EndAction.SCORE_TWO_NOTES && numCenterlineNotesScored == 2)
                    {
                        robot.globalTracer.traceInfo(moduleName, "***** EndAction=" + endAction + ": we are done.");
                        sm.setState(State.DONE);
                    }
                    else
                    {
                        robotPose = robot.robotDrive.driveBase.getFieldPosition();
                        //robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.3);
                        centerlineIndex =
                            startPos == AutoStartPos.SW_CENTER? 1:
                            Math.abs(robotPose.x - RobotParams.Game.WINGNOTE_BLUE_SOURCE_SIDE.x) <
                            Math.abs(robotPose.x - RobotParams.Game.WINGNOTE_BLUE_AMP_SIDE.x)? 0: 2;
                        robot.globalTracer.traceInfo(
                            moduleName, "***** EndAction=" + endAction + ": drive to Centerline " +
                            (centerlineIndex == 0? "Source": centerlineIndex == 2? "Amp": "Center") + ".");
                        targetPose =
                            RobotParams.Game.centerlineNotePickupPoses[centerlineIndex].clone();
                        robot.robotDrive.purePursuitDrive.setWaypointEventHandler(this::waypointHandler);
                        if (startPos != AutoStartPos.AMP)
                        {
                            robot.turtle();
                            // Doing long distance drive, turn on vision guidance.
                            enableAprilTagVision(true);
                            // Waypoint event handler will turn on Note Vision.
                            intermediatePose = targetPose.clone();
                            intermediatePose.y -= 120.0;
                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                robot.adjustPoseByAlliance(intermediatePose, alliance),
                                robot.adjustPoseByAlliance(targetPose, alliance));
                        }
                        else
                        {
                            //Run ultrasonic relocalization
                            double ultrasonicDistance = robot.getUltrasonicDistance();
                            double distanceFromWall = ultrasonicDistance + 11.0;
                            robotPose.x = -RobotParams.Field.WIDTH + distanceFromWall;
                            robot.globalTracer.traceInfo(
                                moduleName,
                                "***** Ultrasonic Relocalize: ultrasonicDistance="+ ultrasonicDistance +
                                ", relocalized robotPose=" + robotPose);
                            robot.robotDrive.setFieldPosition(robotPose, false);
                            // For AutoStartPos.AMP, we want to pick up the Note closest to the wall.
                            targetPose.x = RobotParams.Game.CENTERLINE_NOTE_5.x;
                            intermediatePose = targetPose.clone();
                            intermediatePose.y -= 84.0;
                            intermediatePose.angle = -90.0;
                            intermediatePose2 = targetPose.clone();
                            intermediatePose2.y -= 36.0;
                            intermediatePose2.angle = 200.0;
                            targetPose.angle = 200.0;

                            robot.robotDrive.purePursuitDrive.start(
                                event, robotPose, false,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                                RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                                robot.adjustPoseByAlliance(intermediatePose, alliance),
                                robot.adjustPoseByAlliance(intermediatePose2, alliance),
                                robot.adjustPoseByAlliance(targetPose, alliance));
                        }
                        sm.addEvent(event);
                        // WaypointHandler will turn on NoteVision which will set noteEent.
                        sm.waitForEvents(State.PERFORM_END_ACTION);
                    }
                    break;

                case PERFORM_END_ACTION:
                    robot.globalTracer.traceInfo(moduleName, "***** Turn off Vision and cancel PurePursuitDrive.");
                    robot.robotDrive.purePursuitDrive.cancel();
                    robot.robotDrive.purePursuitDrive.setWaypointEventHandler(null);
                    disableNoteVision();
                    disableAprilTagVision();
                    performingEndAction = true;
                    sm.setState(endAction == EndAction.PARK_NEAR_CENTER_LINE? State.DONE: State.PICKUP_CENTERLINE_NOTE);
                    break;

                case PICKUP_CENTERLINE_NOTE:
                    robot.globalTracer.traceInfo(moduleName, "***** Pickup Centerline Note.");
                    robot.autoPickupFromGround.autoAssistPickup(true, true, event);
                    sm.waitForSingleEvent(
                        event,
                        endAction == EndAction.HOARD_ONE_NOTE? State.PARK:
                        startPos == AutoStartPos.AMP && numWingNotesScored == 0 && numCenterlineNotesScored == 0?
                            State.DRIVE_TO_AMP: State.DRIVE_TO_SPEAKER);
                    break;

                case DRIVE_TO_SPEAKER:
                    robot.globalTracer.traceInfo(moduleName, "***** Drive to Speaker to score Centerline Note.");
                    enableAprilTagVision(true);
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.5);
                    robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    robotPose.y = RobotParams.Game.CENTERLINE_NOTE_5.y;
                    robot.robotDrive.setFieldPosition(robotPose, false);
                    targetPose = RobotParams.Game.centerlineNoteScorePoses[centerlineIndex].clone();
                    intermediatePose = RobotParams.Game.centerlineNotePickupPoses[centerlineIndex].clone();
                    intermediatePose.angle = targetPose.angle;
                    intermediatePose2 = targetPose.clone();
                    intermediatePose2.y += 6.0;
                    intermediatePose2.x += 6.0;
                    robot.robotDrive.purePursuitDrive.setWaypointEventHandler(this::waypointHandler);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robotPose, false,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                        robot.adjustPoseByAlliance(intermediatePose, alliance),
                        robot.adjustPoseByAlliance(intermediatePose2, alliance),
                        robot.adjustPoseByAlliance(targetPose, alliance));
                    sm.waitForSingleEvent(event, State.SCORE_NOTE_TO_SPEAKER);
                    break;

                case DRIVE_TO_AMP:
                    robot.globalTracer.traceInfo(moduleName, "***** Drive to Amp to score Centerline Note.");
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(0.7);
                    // robot.shooter.setTiltAngle(RobotParams.Shooter.tiltTurtleAngle);
                    robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    targetPose = RobotParams.Game.AMP_BLUE_PRESCORE;

                    intermediatePose2 = RobotParams.Game.WINGNOTE_BLUE_AMP_SIDE.clone();
                    intermediatePose2.y += 60.0;
                    intermediatePose2.x -= 24.0;
                    intermediatePose2.angle = -90.0;

                    intermediatePose3 = targetPose.clone();
                    intermediatePose3.x = intermediatePose2.x;

                    intermediatePose = intermediatePose2.clone();
                    intermediatePose.y += 60.0;

                    enableAprilTagVision(false);
                    sm.addEvent(aprilTagEvent);
                    robot.robotDrive.purePursuitDrive.setWaypointEventHandler(this::waypointHandler);
                    robot.robotDrive.purePursuitDrive.start(
                        event, robotPose, false,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                        RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                        robot.adjustPoseByAlliance(intermediatePose, alliance),
                        robot.adjustPoseByAlliance(intermediatePose2, alliance),
                        robot.adjustPoseByAlliance(intermediatePose3, alliance),
                        robot.adjustPoseByAlliance(targetPose, alliance));
                    sm.addEvent(event);
                    sm.waitForEvents(State.SCORE_NOTE_TO_AMP, false);
                    break;

                case PARK:
                    // Make sure we are back on our side if we went over just a little.
                    robot.globalTracer.traceInfo(
                        moduleName, "***** Back up a little to make sure we don't cross Centerline.");
                    robotPose = robot.robotDrive.driveBase.getFieldPosition();
                    double halfFieldLength = RobotParams.Field.LENGTH / 2.0;
                    double halfRobotLength = RobotParams.Robot.LENGTH / 2.0;
                    double robotSafeY = 0.0;

                    if (alliance == Alliance.Red && robotPose.y - halfRobotLength < halfFieldLength)
                    {
                        robotSafeY = halfFieldLength + halfRobotLength + 24.0;
                    }
                    else if (alliance == Alliance.Blue && robotPose.y + halfRobotLength > halfFieldLength)
                    {
                        robotSafeY = halfFieldLength - halfRobotLength - 24.0;
                    }

                    if (robotSafeY != 0.0)
                    {
                        robot.robotDrive.purePursuitDrive.start(
                            event, robotPose, false,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_VELOCITY,
                            RobotParams.SwerveDriveBase.PROFILED_MAX_ACCELERATION,
                            new TrcPose2D(robotPose.x, robotSafeY, robotPose.angle));
                        sm.waitForSingleEvent(event, State.DONE);
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

    /**
     * This method is called when Pure Pursuit crosses a waypoint or the path is completed.
     *
     * @param index specifies the index of the waypoint in the path, -1 if the path is completed or canceled.
     * @param waypoint specifies the current target waypoint.
     */
    private void waypointHandler(int index, TrcWaypoint waypoint)
    {
        State currState = sm.getState();

        robot.globalTracer.traceInfo(
            moduleName, "***** [" + currState + "] Waypoint " + index + ": " + waypoint +
            ", robotPose=" + robot.robotDrive.driveBase.getFieldPosition());
        if (currState == State.DRIVE_TO_WING_NOTE && startPos == AutoStartPos.AMP && index == 1)
        {
            robot.globalTracer.traceInfo(moduleName, "***** Turn on narrow Note Vision.");
            enableNoteVision();
            sm.addEvent(noteEvent);
        }
        else if (currState == State.DRIVE_TO_CENTERLINE && endAction != EndAction.PARK_NEAR_CENTER_LINE)
        {
            // We are driving to Centerline to pick up a Note.
            if (startPos == AutoStartPos.AMP && index == 2)
            {
                // We are picking up a Note closest to the wall while we are turning to face it, turn on narrow
                // vision so we will only stop turning when we are staring right at it.
                robot.globalTracer.traceInfo(moduleName, "***** Turn on narrow Note Vision.");
                enableNoteVision();
                sm.addEvent(noteEvent);
            }
            else if (startPos != AutoStartPos.AMP && index == 1)
            {
                // We are picking up a Note from either SW_SOURCE_SIDE, SW_CENTER or SW_AMP_SIDE and we are at
                // intermediatePose. Turn on full Note Vision.
                robot.globalTracer.traceInfo(moduleName, "***** Turn on full Note Vision.");
                enableNoteVision(
                    RobotParams.Intake.noteDistanceThreshold, RobotParams.Intake.noteFullViewAngle);
                sm.addEvent(noteEvent);
            }
        }
        else if (currState == State.DRIVE_TO_SPEAKER)
        {
            if (index == 1)
            {
                // Prep the shooter in case Vision Tracking doesn't see AprilTag.
                robot.shooter.aimShooter(
                    RobotParams.Shooter.wingNotePresetParams.shooterVelocity,
                    RobotParams.Shooter.wingNotePresetParams.tiltAngle, 0.0);
                robot.globalTracer.traceInfo(moduleName, "***** Turn on AprilTag Vision Tracking.");
                robot.enableAprilTagTracking(4, 7, 3, 8);
                robot.robotDrive.purePursuitDrive.enableFixedHeading(robot::getHeadingOffset);
            }
            else if (index == 2)
            {
                robot.globalTracer.traceInfo(
                    moduleName, "***** Cancel Pursuit and turn off Vision Guidance and Vision Tracking.");
                robot.robotDrive.purePursuitDrive.cancel();
                disableAprilTagVision();
                robot.disableAprilTagTracking();
                robot.robotDrive.purePursuitDrive.disableFixedHeading();
            }
        }
        else if (currState == State.DRIVE_TO_AMP && index == 2)
        {
            if (RobotParams.Preferences.useUltrasonicRelocalization)
            {
                //Run ultrasonic relocalization
                double ultrasonicDistance = robot.getUltrasonicDistance();
                TrcPose2D robotPose = robot.robotDrive.driveBase.getFieldPosition();
                double distanceFromWall = ultrasonicDistance + 11.0;
                robotPose.x = -RobotParams.Field.WIDTH + distanceFromWall;
                robot.globalTracer.traceInfo(
                    moduleName,
                    "***** Ultrasonic Relocalize: ultrasonicDistance="+ ultrasonicDistance + ", relocalized robotPose=" + robotPose);
                robot.robotDrive.setFieldPosition(robotPose, false);
            }
        }
    }   //waypointHandler

    /**
     * This method enables AprilTag detection to signal an event so that Auto can interrupt PurePursuitDrive, for
     * example.
     *
     * @param visionGuidance specifies true to enable vision guidance, false to disable.
     */
    private void enableAprilTagVision(boolean visionGuidance)
    {
        aprilTagVisionEnabled = true;
        visionGuidanceEnabled = visionGuidance;
    }   //enableAprilTagVision

    /**
     * This method disables AprilTag detection.
     */
    private void disableAprilTagVision()
    {
        aprilTagVisionEnabled = false;
        visionGuidanceEnabled = false;
    }   //disableAprilTagVision

    /**
     * This method enables Note detection to signal an event so that Auto can interrupt PurePursuitDrive, for example.
     *
     * @param distanceThreshold specifies the distance threshold within which to signal Note detected.
     * @param angleThreshold specifies the angle threshold within which to signal Note detected.
     */
    private void enableNoteVision(double distanceThreshold, double angleThreshold)
    {
        noteVisionEnabled = true;
        noteDistanceThreshold = distanceThreshold;
        noteAngleThreshold = angleThreshold;
    }   //enableNoteVision

    /**
     * This method enables Note detection to signal an event so that Auto can interrupt PurePursuitDrive, for example.
     */
    private void enableNoteVision()
    {
        noteVisionEnabled = true;
        noteDistanceThreshold = RobotParams.Intake.noteDistanceThreshold;
        noteAngleThreshold = RobotParams.Intake.noteAngleThreshold;
    }   //enableNoteVision

    /**
     * This method disables Note detection.
     */
    private void disableNoteVision()
    {
        noteVisionEnabled = false;
        noteDistanceThreshold = RobotParams.Intake.noteDistanceThreshold;
        noteAngleThreshold = RobotParams.Intake.noteAngleThreshold;
    }   //disableNoteVision

}   //class CmdAuto
