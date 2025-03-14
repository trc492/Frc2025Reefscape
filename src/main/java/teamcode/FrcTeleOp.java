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

import frclib.driverio.FrcXboxController;
import frclib.vision.FrcPhotonVision.DetectedObject;
import teamcode.subsystems.CoralArm;
import teamcode.subsystems.Elevator;
import teamcode.subsystems.Winch;
import teamcode.vision.PhotonVision.PipelineType;
import trclib.drivebase.TrcSwerveDriveBase;
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
    //
    // Global objects.
    //
    protected final Robot robot;
    private double driveSpeedScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
    private double turnSpeedScale = RobotParams.Robot.TURN_NORMAL_SCALE;
    private boolean controlsEnabled = false;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    private boolean subsystemStatusOn = true;
    private boolean relocalizing = false;
    private TrcPose2D robotFieldPose = null;
    private double prevCoralArmPower = 0.0;
    private double prevElevatorPower = 0.0;
    private double prevWinchPower = 0.0;
    private int scoreIndex = 3;

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
            robot.setDriveOrientation(RobotParams.Robot.DRIVE_ORIENTATION, false);
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
        robot.cancelAll();
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
    @SuppressWarnings("unused")
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        int lineNum = 1;

        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                //
                // DriveBase operation.
                //
                if (robot.robotDrive != null)
                {
                    boolean showDriveBaseStatus =
                        RobotParams.Preferences.showDriveBase &&
                        (RobotParams.Preferences.doStatusUpdate || subsystemStatusOn);
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
                            RobotParams.Robot.DRIVE_MODE, true, driveSpeedScale, turnSpeedScale);
                        if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                        {
                            double gyroAngle = robot.robotDrive.driveBase.getDriveGyroAngle();
                            if ((driveInputs[0] != 0.0 || driveInputs[1] != 0.0) &&
                                robot.pickupCoralFromStationTask != null &&
                                robot.pickupCoralFromStationTask.isActive())
                            {
                                robot.pickupCoralFromStationTask.cancel();
                            }
                            robot.robotDrive.driveBase.holonomicDrive(
                                null, driveInputs[0], driveInputs[1], driveInputs[2], gyroAngle);
                            if (showDriveBaseStatus)
                            {
                                robot.dashboard.displayPrintf(
                                    lineNum, "Holonomic: x=%.2f, y=%.2f, rot=%.2f, gyroAngle=%.2f",
                                    driveInputs[0], driveInputs[1], driveInputs[2], gyroAngle);
                            }
                        }
                        else
                        {
                            robot.robotDrive.driveBase.arcadeDrive(driveInputs[1], driveInputs[2]);
                            if (showDriveBaseStatus)
                            {
                                robot.dashboard.displayPrintf(
                                    lineNum, "Arcade: x=%.2f, y=%.2f, rot=%.2f",
                                    driveInputs[0], driveInputs[1], driveInputs[2]);
                            }
                        }
                    }

                    if (showDriveBaseStatus)
                    {
                        lineNum++;
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

                    if (robot.winch != null)
                    {
                        power = robot.driverController.getTrigger(true) * Winch.Params.POWER_LIMIT;
                        if (power != prevWinchPower)
                        {
                            //robot.winch.setPower(power);
                            robot.winch.setPidPower(power, Winch.Params.MIN_POS, Winch.Params.MAX_POS, true);
                            prevWinchPower = power;
                        }
                    }
                }
            }
            //
            // Update robot status.
            //
            if (RobotParams.Preferences.doStatusUpdate || subsystemStatusOn)
            {
                lineNum = robot.updateStatus(lineNum);
            }
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

                // This does not work!!
                // if (robot.pickupCoralFromStationTask != null)
                // {
                //     if (pressed)
                //     {
                //         robot.pickupCoralFromStationTask.autoPickupCoral(null, true, -1, false, null);
                //     }
                //     else
                //     {
                //         robot.pickupCoralFromStationTask.cancel();
                //     }
                // } 
                break;

            case X:
                if (robot.robotDrive != null && pressed)
                {
                    ((TrcSwerveDriveBase) (robot.robotDrive.driveBase)).setXMode(null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> X Mode");
                }
                break;

            case Y:
                if (robot.pickupCoralFromStationTask != null && pressed)
                {
                    robot.pickupCoralFromStationTask.autoPickupCoral(moduleName, true, -1, false, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Auto Pickup Coral");
                } else{
                    
                }
                break;  

            case LeftBumper:
                driverAltFunc = pressed;
                robot.globalTracer.traceInfo(moduleName, ">>>>> DriverAltFunc = " + driverAltFunc);
                break;

            case RightBumper:
                if (pressed)
                {
                    driveSpeedScale = RobotParams.Robot.DRIVE_SLOW_SCALE;
                    turnSpeedScale = RobotParams.Robot.TURN_SLOW_SCALE;
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Slow Drive");
                }
                else
                {
                    driveSpeedScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
                    turnSpeedScale = RobotParams.Robot.TURN_NORMAL_SCALE;
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Normal Drive");
                }
                break;

            case DpadUp:
                if (robot.climbTask != null && pressed)
                {
                    robot.winch.zeroCalibrate(Winch.Params.ZERO_CAL_POWER);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Zero Calibrate Climb");
                }
                break;

            case DpadDown:  
                if (robot.climbTask != null && pressed)
                {
                    robot.climbTask.climb(null, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Climb");
                }   
                break;

            case DpadLeft:
                if (robot.climbTask != null && pressed)
                {
                    robot.climbTask.deployClimber(null, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Zero Calibrate Climber and Extend");
                }
                break;

            case DpadRight:
                if (robot.climbTask != null && pressed)
                {
                    robot.climbTask.prepClimber(null, null);
                    robot.globalTracer.traceInfo(moduleName, ">>>>> Engage Cage");
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
                if (driverAltFunc)
                {
                    if (pressed)
                    {
                        subsystemStatusOn = !subsystemStatusOn;
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Toggle Subsystem Status: status=" + subsystemStatusOn);
                    }
                }
                else
                {
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
                                    moduleName, ">>>>> Finish re-localizing: AprilTag not found.");
                            }
                        }
                        else
                        {
                            robot.globalTracer.traceInfo(moduleName, ">>>>> Start re-localizing ...");
                        }
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
                    else if (robot.coralGrabber.isAutoActive())
                    {
                        robot.coralGrabber.cancel();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Auto Coral Intake");
                    }
                    else
                    {
                        robot.coralGrabber.stop();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Stop Coral Intake");
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
                    else if (robot.coralGrabber.isAutoActive())
                    {
                        robot.coralGrabber.cancel();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Cancel Auto Coral Eject");
                    }
                    else
                    {
                        robot.coralGrabber.stop();
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Stop Coral Eject");
                    }
                }
                break;

            case X:
                if (robot.elevatorArmTask !=null && pressed)
                {
                    robot.elevatorArmTask.setCoralScorePosition(moduleName, scoreIndex, null);
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
                robot.globalTracer.traceInfo(moduleName, ">>>>> OperatorAltFunc = " + driverAltFunc);
                break;

            case RightBumper:
                break;

            case DpadUp:
                if (pressed)
                {
                    if (scoreIndex < 3)
                    {
                        scoreIndex++;
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Score Index Up: index=", scoreIndex);
                    }

                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setReefLevel(scoreIndex);
                    }
                }
                break;

            case DpadDown:
                if (pressed)
                {
                    if (scoreIndex > 0)
                    {
                        scoreIndex--;
                        robot.globalTracer.traceInfo(moduleName, ">>>>> Score Index Down: index=", scoreIndex);
                    }

                    if (robot.ledIndicator != null)
                    {
                        robot.ledIndicator.setReefLevel(scoreIndex);
                    }
                }
                break;

            case DpadLeft:
                break;

            case DpadRight:
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
