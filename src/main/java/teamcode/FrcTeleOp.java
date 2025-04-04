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

package teamcode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcXboxController;
import frclib.vision.FrcPhotonVision.DetectedObject;
import teamcode.subsystems.Climber;
import teamcode.subsystems.CoralArm;
import teamcode.subsystems.Elevator;
import teamcode.tasks.TaskAutoScoreCoral.ScoreCoralOffset;
import teamcode.vision.PhotonVision.PipelineType;
import trclib.drivebase.TrcSwerveDriveBase;
import trclib.driverio.TrcGameController.DriveMode;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    private static final String moduleName = FrcTeleOp.class.getSimpleName();
    protected static final boolean traceButtonEvents = true;

    private static final String DBKEY_DRIVE_MODE = "TeleOp/DriveMode";                  //Choices
    private static final String DBKEY_DRIVE_ORIENTATION = "TeleOp/DriveOrientation";    //Choices
    private static final String DBKEY_DRIVE_NORMAL_SCALE = "TeleOp/DriveNormalScale";   //Number
    private static final String DBKEY_DRIVE_SLOW_SCALE = "TeleOp/DriveSlowScale";       //Number
    private static final String DBKEY_TURN_NORMAL_SCALE = "TeleOp/TurnNormalScale";     //Number
    private static final String DBKEY_TURN_SLOW_SCALE = "TeleOp/TurnSlowScale";         //Number
    private static final String DBKEY_SHOW_DRIVE_POWER = "TeleOp/ShowDrivePower";       //Boolean
    private static final String DBKEY_DRIVE_POWER = "TeleOp/DrivePower";                //String
    private static final double DEF_DRIVE_NORMAL_SCALE = 1.0;
    private static final double DEF_DRIVE_SLOW_SCALE = 0.175;
    private static final double DEF_TURN_NORMAL_SCALE = 0.6;
    private static final double DEF_TURN_SLOW_SCALE = 0.15;
    private static final ScoreCoralOffset[] leftScoreOffsets = new ScoreCoralOffset[]
    {
        new ScoreCoralOffset(0.0, 0.0),
        new ScoreCoralOffset(-6.0, -16.0),
        new ScoreCoralOffset(-4.0, -18.0),
        new ScoreCoralOffset(-2.0, -19.0)
    };
    private static final ScoreCoralOffset[] rightScoreOffsets = new ScoreCoralOffset[]
    {
        new ScoreCoralOffset(0.0, 0.0),
        new ScoreCoralOffset(7.5, -16.0),
        new ScoreCoralOffset(9.0, -17.5),
        new ScoreCoralOffset(7.5, -19.0)
    };

    //
    // Global objects.
    //
    protected final Robot robot;
    private final FrcChoiceMenu<DriveMode> driveModeMenu;
    private final FrcChoiceMenu<DriveOrientation> driveOrientationMenu;
    private double driveSpeedScale;
    private double turnSpeedScale;
    private boolean controlsEnabled = false;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    private boolean relocalizing = false;
    private TrcPose2D robotFieldPose = null;
    private double prevCoralArmPower = 0.0;
    private double prevElevatorPower = 0.0;
    private double prevClimberArmPower = 0.0;
    private int scoreLevelIndex = 3;
    private boolean scoreRightSide = true;
    private boolean rumbling = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param robot specifies the robot object to access all robot hardware and subsystems.
     */
    public FrcTeleOp(Robot robot)
    {
        //
        // Create and initialize global object.
        //
        this.robot = robot;

        driveModeMenu = new FrcChoiceMenu<>(DBKEY_DRIVE_MODE);
        driveModeMenu.addChoice("Tank", DriveMode.TankMode);
        driveModeMenu.addChoice("Holonomic", DriveMode.HolonomicMode);
        driveModeMenu.addChoice("Arcade", DriveMode.ArcadeMode, true, true);

        driveOrientationMenu = new FrcChoiceMenu<>(DBKEY_DRIVE_ORIENTATION);
        driveOrientationMenu.addChoice("Inverted", DriveOrientation.INVERTED);
        driveOrientationMenu.addChoice("Robot", DriveOrientation.ROBOT);
        driveOrientationMenu.addChoice("Field", DriveOrientation.FIELD, true, true);

        robot.dashboard.refreshKey(DBKEY_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
        robot.dashboard.refreshKey(DBKEY_DRIVE_SLOW_SCALE, DEF_DRIVE_SLOW_SCALE);
        robot.dashboard.refreshKey(DBKEY_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
        robot.dashboard.refreshKey(DBKEY_TURN_SLOW_SCALE, DEF_TURN_SLOW_SCALE);
        robot.dashboard.refreshKey(DBKEY_SHOW_DRIVE_POWER, RobotParams.Preferences.showDrivePower);
        robot.dashboard.refreshKey(DBKEY_DRIVE_POWER, "");

        driveSpeedScale = robot.dashboard.getNumber(DBKEY_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
        turnSpeedScale = robot.dashboard.getNumber(DBKEY_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
}   //FrcTeleOp

    //
    // Implements TrcRobot.RunMode interface.
    //

    /**
     * This method is called when the teleop mode is about to start. Typically, you put code that will prepare
     * the robot for start of teleop here such as creating and configuring joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Enabling joysticks.
        //
        setControlsEnabled(true);
        //
        // Initialize subsystems for TeleOp mode if necessary.
        //
        if (robot.robotDrive != null)
        {
            // Set robot to FIELD by default but don't change the heading.
            robot.setDriveOrientation(driveOrientationMenu.getCurrentChoiceObject(), false);
            // Enable AprilTag vision for re-localization.
            if (robot.photonVisionFront != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling FrontCam for AprilTagVision.");
                robot.photonVisionBack.setPipeline(PipelineType.APRILTAG);
            }

            if (robot.photonVisionBack != null)
            {
                robot.globalTracer.traceInfo(moduleName, "Enabling BackCam for AprilTagVision.");
                robot.photonVisionBack.setPipeline(PipelineType.APRILTAG);
            }
        }

        if (RobotParams.Preferences.hybridMode)
        {
            // This makes sure that the autonomous stops running when
            // teleop starts running. If you want the autonomous to
            // continue until interrupted by another command, remove
            // this line or comment it out.
            if (robot.m_autonomousCommand != null)
            {
                robot.m_autonomousCommand.cancel();
            }
        }
    }   //startMode

    /**
     * This method is called when teleop mode is about to end. Typically, you put code that will do clean
     * up here such as disabling joysticks and other subsystems.
     *
     * @param prevMode specifies the previous RunMode it is coming from.
     * @param nextMode specifies the next RunMode it is going into.
     */
    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        //
        // Disabling joysticks.
        //
        setControlsEnabled(false);
        //
        // Disable subsystems before exiting if necessary.
        //
    }   //stopMode

    /**
     * This method is called periodically on the main robot thread. Typically, you put TeleOp control code here that
     * doesn't require frequent update For example, TeleOp joystick code or status display code can be put here since
     * human responses are considered slow.
     *
     * @param elapsedTime specifies the elapsed time since the mode started.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                //
                // DriveBase operation.
                //
                if (robot.robotDrive != null)
                {
                    boolean showDriveBaseStatus = robot.dashboard.getBoolean(
                        DBKEY_DRIVE_POWER, RobotParams.Preferences.showDrivePower);
                    if (relocalizing)
                    {
                        if (robotFieldPose == null)
                        {
                            DetectedObject aprilTagObj = null;

                            if (robot.photonVisionFront != null)
                            {
                                aprilTagObj = robot.photonVisionFront.getBestDetectedAprilTag(null);
                            }

                            if (aprilTagObj == null && robot.photonVisionBack != null)
                            {
                                aprilTagObj = robot.photonVisionBack.getBestDetectedAprilTag(null);
                            }

                            if (aprilTagObj != null)
                            {
                                robotFieldPose = robot.photonVisionBack.getRobotFieldPose(aprilTagObj, false);
                            }
                        }
                    }
                    else
                    {
                        double[] driveInputs;

                        driveInputs = robot.driverController.getDriveInputs(
                            driveModeMenu.getCurrentChoiceObject(), true, driveSpeedScale, turnSpeedScale);
                        if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                        {
                            double gyroAngle = robot.robotDrive.driveBase.getDriveGyroAngle();
                            if (driveInputs[0] != 0.0 || driveInputs[1] != 0.0)
                            {
                                if (robot.pickupCoralFromStationTask != null && robot.pickupCoralFromStationTask.isActive())
                                {
                                    robot.pickupCoralFromStationTask.cancel();
                                }

                                if (robot.scoreCoralTask != null && robot.scoreCoralTask.isActive())
                                {
                                    robot.scoreCoralTask.cancel();
                                }
                            }

                            robot.robotDrive.driveBase.holonomicDrive(
                                null, driveInputs[0], driveInputs[1], driveInputs[2], gyroAngle);
                            if (showDriveBaseStatus)
                            {
                                robot.dashboard.putString(
                                    DBKEY_DRIVE_POWER,
                                    String.format(
                                        "Holonomic: x=%.2f, y=%.2f, rot=%.2f, gyroAngle=%.2f",
                                        driveInputs[0], driveInputs[1], driveInputs[2], gyroAngle));
                            }
                        }
                        else
                        {
                            robot.robotDrive.driveBase.arcadeDrive(driveInputs[1], driveInputs[2]);
                            if (showDriveBaseStatus)
                            {
                                robot.dashboard.putString(
                                    DBKEY_DRIVE_POWER,
                                    String.format(
                                        "Arcade: x=%.2f, y=%.2f, rot=%.2f",
                                        driveInputs[0], driveInputs[1], driveInputs[2]));
                            }
                        }
                    }
                }
                //
                // Analog control of subsystem is done here if necessary.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                    double power;

                    if (robot.elevatorArmTask != null)
                    {
                        power = robot.operatorController.getLeftStickY(true) * CoralArm.Params.POWER_LIMIT;
                        if (power != prevCoralArmPower)
                        {
                            if (operatorAltFunc)
                            {
                                robot.elevatorArmTask.setCoralArmPower(null, power);
                            }
                            else
                            {
                                robot.elevatorArmTask.setCoralArmPidPower(null, power);
                            }
                            prevCoralArmPower = power;
                        }

                        power = robot.operatorController.getRightStickY(true) * Elevator.Params.POWER_LIMIT;
                        if (power != prevElevatorPower)
                        {
                            if (operatorAltFunc)
                            {
                                robot.elevatorArmTask.setElevatorPower(null, power);
                            }
                            else
                            {
                                robot.elevatorArmTask.setElevatorPidPower(null, power);
                            }
                            prevElevatorPower = power;
                        }
                    }

                    if (robot.climber != null)
                    {
                        power = robot.driverController.getTrigger(true) * Climber.ArmParams.POWER_LIMIT;
                        if (power != prevClimberArmPower)
                        {
                            if (driverAltFunc)
                            {
                                robot.climber.setPower(power);
                            }
                            else
                            {
                                robot.climber.setPidPower(
                                    power, Climber.ArmParams.MIN_POS, Climber.ArmParams.MAX_POS, true);
                            }
                            prevClimberArmPower = power;
                        }
                    }
                }

                if (RobotParams.Preferences.useRumble && !rumbling &&
                    elapsedTime > RobotParams.Game.TELEOP_PERIOD - RobotParams.Game.CLIMB_PERIOD)
                {
                    robot.driverController.setRumble(RumbleType.kBothRumble, 1.0, 0.5);
                    rumbling = true;
                }
            }
            //
            // Update robot status.
            //
            Dashboard.updateDashboard(robot, 1);
        }
    }   //periodic

    /**
     * This method enables/disables joystick controls.
     *
     * @param enabled specifies true to enable joystick control, false to disable.
     */
    protected void setControlsEnabled(boolean enabled)
    {
        controlsEnabled = enabled;
        robot.driverController.setButtonEventHandler(enabled? this::driverControllerButtonEvent: null);
        robot.operatorController.setButtonEventHandler(enabled? this::operatorControllerButtonEvent: null);
    }   //setControlsEnabled

    //
    // Implements FrcButtonHandler.
    //

    /**
     * This method is called when a driver controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "DriverController: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            case A:
                // Toggle between field or robot oriented driving.
                if (robot.robotDrive != null && pressed)
                {
                    if (driverAltFunc)
                    {
                        if (robot.robotDrive.driveBase.getDriveOrientation() != DriveOrientation.FIELD)
                        {
                            robot.setDriveOrientation(DriveOrientation.FIELD, true);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Setting Mode to: Field");
                        }
                        else
                        {
                            robot.setDriveOrientation(DriveOrientation.ROBOT, false);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Setting Mode to: Robot");
                        }
                    }
                    else
                    {
                        robot.robotDrive.driveBase.resetFieldForwardHeading();
                        robot.globalTracer.traceInfo(
                            moduleName,
                            ">>>>> Reset field forward heading (heading=" + robot.robotDrive.driveBase.getHeading() +
                            ")");
                    }
                }
                break;

            case B:
                // Turtle mode.
                if (pressed)
                {
                    robot.turtle();
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Turtle Mode");
                }
                break;

            case X:
                if (robot.robotDrive != null && pressed)
                {
                    ((TrcSwerveDriveBase) (robot.robotDrive.driveBase)).setXMode(null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> X Mode");
                }
                break;

            case Y:
                if (driverAltFunc)
                {
                    if (robot.scoreCoralTask != null && pressed)
                    {
                        robot.scoreCoralTask.autoScoreCoral(
                            moduleName, true, -1, scoreLevelIndex, scoreRightSide, false, false, true, 0.4,
                            scoreRightSide? rightScoreOffsets[scoreLevelIndex]: leftScoreOffsets[scoreLevelIndex], null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Auto Align to Score Coral");
                    }
                }
                else
                {
                    if (robot.pickupCoralFromStationTask != null && pressed)
                    {
                        robot.pickupCoralFromStationTask.autoPickupCoral(moduleName, true, -1, false, true, null);
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Auto Pickup Coral");
                    }
                }
                break;  

            case LeftBumper:
                driverAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc=" + driverAltFunc);
                break;

            case RightBumper:
                if (pressed)
                {
                    driveSpeedScale = robot.dashboard.getNumber(DBKEY_DRIVE_SLOW_SCALE, DEF_DRIVE_SLOW_SCALE);
                    turnSpeedScale = robot.dashboard.getNumber(DBKEY_TURN_SLOW_SCALE, DEF_TURN_SLOW_SCALE);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Slow Drive");
                }
                else
                {
                    driveSpeedScale = robot.dashboard.getNumber(DBKEY_DRIVE_NORMAL_SCALE, DEF_DRIVE_NORMAL_SCALE);
                    turnSpeedScale = robot.dashboard.getNumber(DBKEY_TURN_NORMAL_SCALE, DEF_TURN_NORMAL_SCALE);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Normal Drive");
                }
                break;

            case DpadRight:
                // Intentional Fall through
            case DpadUp:
                if (robot.climber != null && pressed)
                {
                    robot.elevatorArmTask.coralArm.setPosition(CoralArm.Params.CLIMB_POS);
                    robot.climber.deploy(moduleName);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Deploy Climber");
                }
                break;

            case DpadLeft:
                // Intentional Fall Through
            case DpadDown:
                if (robot.climber != null && pressed)
                {
                    robot.climber.climb(moduleName);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Climb");
                }
                break;  

            // case DpadLeft:
            //     if (robot.climber != null && pressed)
            //     {
            //         robot.climber.resetState();
            //         robot.globalTracer.traceInfo(moduleName, ">>>>> Climber Turtle");
            //     }
            //     break;

            // case DpadRight:
            //     if (robot.climber.grabber != null)
            //     {
            //         if(pressed)
            //         {
            //             robot.climber.grabber.autoIntake(null);
            //         }
            //         else
            //         {
            //             robot.climber.cancel();
            //         }
            //     }
            //     break;

            case Back:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.zeroCalibrate(null, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All and Zero Calibrate");
                }
                break;

            case Start:
                if (robot.photonVisionFront != null &&
                    robot.photonVisionFront.getPipeline() == PipelineType.APRILTAG ||
                    robot.photonVisionBack != null &&
                    robot.photonVisionBack.getPipeline() == PipelineType.APRILTAG)
                {
                    // On press of the button, we will start looking for AprilTag for re-localization.
                    // On release of the button, we will set the robot's field location if we found the
                    // AprilTag.
                    relocalizing = pressed;
                    if (!pressed)
                    {
                        if (robotFieldPose != null)
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, ">>>>> Finish re-localizing: pose=" + robotFieldPose);
                            robot.robotDrive.driveBase.setFieldPosition(robotFieldPose, false);
                            robotFieldPose = null;
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(
                                moduleName, ">>>>> Finish re-localizing: AprilTag not found");
                        }
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Start re-localizing ...");
                    }
                }
                break;

            default:
                break;
        }
    }   //driverControllerButtonEvent

    /**
     * This method is called when an operator controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            15, "OperatorController: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            case A:
                if (robot.coralGrabber!= null)
                {
                    if (pressed)
                    {
                        if (operatorAltFunc)
                        {
                            robot.coralGrabber.intake(0.0, null);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Manual Coral Intake");
                        }
                        else
                        {
                            robot.coralGrabber.autoIntake(null);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Auto Coral Intake");
                        }
                    }
                    else
                    {
                        robot.coralGrabber.cancel();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Coral Intake");
                    }
                }
                break;

            case B:
                if (robot.coralGrabber!= null)
                {
                    if (pressed)
                    {
                        if (operatorAltFunc)
                        {
                            robot.coralGrabber.eject(0.0, null);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Manual Coral Eject");
                        }
                        else
                        {
                            robot.coralGrabber.autoEject(null, 0.5, null, 0.0);
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Auto Coral Eject");
                        }
                    }
                    else
                    {
                        robot.coralGrabber.cancel();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Coral Eject");
                    }
                }
                break;

            case X:
                if (robot.elevatorArmTask !=null && pressed)
                {
                    robot.elevatorArmTask.setCoralScorePosition(moduleName, scoreLevelIndex, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Set Coral Score Position");
                }
                break;

            case Y:
                if (robot.elevatorArmTask != null && pressed)
                {
                    robot.elevatorArmTask.setCoralStationPickupPosition(moduleName, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Set Coral Station Pickup Position");
                }
                break;

            case LeftBumper:
                operatorAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc=" + operatorAltFunc);
                break;

            case RightBumper:
                break;

            case DpadUp:
                if (pressed)
                {
                    if (scoreLevelIndex < 3)
                    {
                        scoreLevelIndex++;
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Score Index Up: index=", scoreLevelIndex);
                    }

                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setReefLevel(scoreLevelIndex);
                    }
                }
                break;

            case DpadDown:
                if (pressed)
                {
                    if (scoreLevelIndex > 0)
                    {
                        scoreLevelIndex--;
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Score Index Down: index=", scoreLevelIndex);
                    }

                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setReefLevel(scoreLevelIndex);
                    }
                }
                break;

            case DpadLeft:
                if (pressed)
                {
                    scoreRightSide = false;
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Score Left Reef Branch");
                }
                break;

            case DpadRight:
                if (pressed)
                {
                    scoreRightSide = true;
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Score Right Reef Branch");
                }
                break;

            case Back:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.zeroCalibrate(null, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All and Zero Calibrate");
                }
                break;

            case Start:
                if (pressed)
                {
                    robot.cancelAll();
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel All");
                }
                break;

            default:
                break;
        }
    }   //operatorControllerButtonEvent

}   //class FrcTeleOp
