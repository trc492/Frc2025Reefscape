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

import frclib.driverio.FrcButtonPanel;
import frclib.driverio.FrcDualJoystick;
import frclib.driverio.FrcJoystick;
import frclib.driverio.FrcXboxController;
import frclib.vision.FrcPhotonVision.DetectedObject;
import teamcode.subsystems.AlgaeArm;
import teamcode.subsystems.CoralArm;
import teamcode.subsystems.Elevator;
import teamcode.vision.PhotonVision.PipelineType;
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
    private double prevAlgaeArmPower = 0.0;
    private double prevElevatorPower = 0.0;
    private int scoreIndex = 1;

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
            robot.robotDrive.driveBase.setDriveOrientation(DriveOrientation.FIELD, true);
            // Enable AprilTag vision for re-localization.
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
    @Override
    public void periodic(double elapsedTime, boolean slowPeriodicLoop)
    {
        int lineNum = 1;

        if (slowPeriodicLoop)
        {
            if (controlsEnabled)
            {
                robot.dashboard.displayPrintf(lineNum++, "Coral Score Level=%s", scoreIndex);
                //
                // DriveBase operation.
                //
                if (robot.robotDrive != null)
                {
                    if (relocalizing)
                    {
                        if (robotFieldPose == null)
                        {
                            DetectedObject aprilTagObj = robot.photonVisionBack.getBestDetectedAprilTag(null);
                            if (aprilTagObj != null)
                            {
                                robotFieldPose = robot.photonVisionBack.getRobotFieldPose(aprilTagObj, false);
                            }
                        }
                    }
                    else
                    {
                        double[] driveInputs;

                        if (robot.driverController != null)
                        {
                            driveInputs = robot.driverController.getDriveInputs(
                                RobotParams.Robot.DRIVE_MODE, true, driveSpeedScale, turnSpeedScale);
                        }
                        else if (robot.driverJoystick != null)
                        {
                            driveInputs = robot.driverJoystick.getDriveInputs(
                                RobotParams.Robot.DRIVE_MODE, true, driveSpeedScale, turnSpeedScale);
                        }
                        else
                        {
                            driveInputs = robot.driverDualJoystick.getDriveInputs(
                                RobotParams.Robot.DRIVE_MODE, true, driveSpeedScale, turnSpeedScale);
                        }

                        if (robot.robotDrive.driveBase.supportsHolonomicDrive())
                        {
                            double gyroAngle = robot.robotDrive.driveBase.getDriveGyroAngle();
                            robot.robotDrive.driveBase.holonomicDrive(
                                null, driveInputs[0], driveInputs[1], driveInputs[2], gyroAngle);
                            if (subsystemStatusOn)
                            {
                                robot.dashboard.displayPrintf(
                                    lineNum++, "Holonomic: x=%.2f, y=%.2f, rot=%.2f, gyroAngle=%.2f",
                                    driveInputs[0], driveInputs[1], driveInputs[2], gyroAngle);
                            }
                        }
                        else if (RobotParams.Preferences.useTankDrive)
                        {
                            robot.robotDrive.driveBase.tankDrive(driveInputs[0], driveInputs[1]);
                            if (subsystemStatusOn)
                            {
                                robot.dashboard.displayPrintf(
                                    lineNum++, "Tank: left=%.2f, right=%.2f, rot=%.2f",
                                    driveInputs[0], driveInputs[1], driveInputs[2]);
                            }
                        }
                        else
                        {
                            robot.robotDrive.driveBase.arcadeDrive(driveInputs[1], driveInputs[2]);
                            if (subsystemStatusOn)
                            {
                                robot.dashboard.displayPrintf(
                                    lineNum++, "Arcade: x=%.2f, y=%.2f, rot=%.2f",
                                    driveInputs[0], driveInputs[1], driveInputs[2]);
                            }
                        }

                        if (subsystemStatusOn)
                        {
                            robot.dashboard.displayPrintf(
                                lineNum++, "RobotPose=%s, Orient=%s, GyroAssist=%s",
                                robot.robotDrive.driveBase.getFieldPosition(),
                                robot.robotDrive.driveBase.getDriveOrientation(),
                                robot.robotDrive.driveBase.isGyroAssistEnabled());
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

                        power = robot.operatorController.getTrigger(true) * AlgaeArm.Params.POWER_LIMIT;
                        if (power != prevAlgaeArmPower)
                        {
                            if (operatorAltFunc)
                            {
                                robot.elevatorArmTask.setAlgaeArmPower(null, power);
                            }
                            else
                            {
                                robot.elevatorArmTask.setAlgaeArmPidPower(null, power);
                            }
                            prevAlgaeArmPower = power;
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
                    else
                    {
                        if (robot.coralArm != null)
                        {
                            power = robot.operatorController.getLeftStickY(true) * CoralArm.Params.POWER_LIMIT;
                            if (power != prevCoralArmPower)
                            {
                                if (operatorAltFunc)
                                {
                                    robot.coralArm.setPower(power);
                                }
                                else
                                {
                                    robot.coralArm.setPidPower(
                                        power, CoralArm.Params.MIN_POS, CoralArm.Params.MAX_POS, true);
                                }
                                prevCoralArmPower = power;
                            }
                        }

                        if (robot.algaeArm != null)
                        {
                            power = robot.operatorController.getTrigger(true) * AlgaeArm.Params.POWER_LIMIT;
                            if (power != prevAlgaeArmPower)
                            {
                                if (operatorAltFunc)
                                {
                                    robot.algaeArm.setPower(power);
                                }
                                else
                                {
                                    robot.algaeArm.setPidPower(
                                        power, AlgaeArm.Params.MIN_POS, AlgaeArm.Params.MAX_POS, true);
                                }
                                prevAlgaeArmPower = power;
                            }
                        }

                        if (robot.elevator != null)
                        {
                            power = robot.operatorController.getRightStickY(true) * Elevator.Params.POWER_LIMIT;
                            if (power != prevElevatorPower)
                            {
                                if (operatorAltFunc)
                                {
                                    robot.elevator.setPower(power);
                                }
                                else
                                {
                                    robot.elevator.setPidPower(
                                        power, Elevator.Params.MIN_POS, Elevator.Params.MAX_POS, true);
                                }
                                prevElevatorPower = power;
                            }
                        }
                    }
                }
            }
            //
            // Update robot status.
            //
            if (RobotParams.Preferences.doStatusUpdate)
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

        if (robot.driverController != null)
        {
            robot.driverController.setButtonEventHandler(enabled? this::driverControllerButtonEvent: null);
        }
        else if (robot.driverJoystick != null)
        {
            robot.driverJoystick.setButtonEventHandler(enabled? this::driverJoystickButtonEvent: null);
        }
        else
        {
            robot.driverDualJoystick.setButtonEventHandler(enabled? this::driverDualJoystickButtonEvent: null);
        }

        if (robot.operatorController != null)
        {
            robot.operatorController.setButtonEventHandler(enabled? this::operatorControllerButtonEvent: null);
        }
        else
        {
            robot.operatorStick.setButtonEventHandler(enabled? this::operatorStickButtonEvent: null);
        }

        if (RobotParams.Preferences.useButtonPanels)
        {
            robot.buttonPanel.setButtonEventHandler(enabled? this::buttonPanelButtonEvent: null);
            robot.switchPanel.setButtonEventHandler(enabled? this::switchPanelButtonEvent: null);
        }
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
            8, "DriverController: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            case A:
                // Toggle between field or robot oriented driving.
                if (robot.robotDrive != null && pressed)
                {
                    if (robot.robotDrive.driveBase.getDriveOrientation() != DriveOrientation.FIELD)
                    {
                        robot.globalTracer.traceInfo(moduleName, "Setting Mode to: Field");
                        robot.setDriveOrientation(DriveOrientation.FIELD, true);
                    }
                    else
                    {
                        robot.globalTracer.traceInfo(moduleName, "Setting Mode to: Robot");
                        robot.setDriveOrientation(DriveOrientation.ROBOT, false);
                    }
                }
                break;

            case B:
                // Turtle mode.
                if (pressed)
                {
                    robot.turtle();
                }
                break;

            case X:
                if (robot.robotDrive != null && pressed && robot.scoreCoralTask != null)
                {
                    robot.scoreCoralTask.autoScoreCoral(
                        moduleName, true, 0, false, false, false, 0, null);
                }
                else
                {
                    robot.globalTracer.traceInfo(moduleName, "AutoScoreState: " + robot.scoreCoralTask);
                }
                break;

            case Y:
                if(robot.robotDrive != null && pressed && robot.pickupCoralFromStationTask != null)
                {
                    robot.pickupCoralFromStationTask.autoPickupCoral(
                        moduleName, true, false, false, null);
                }
                break;  

            case LeftBumper:
                driverAltFunc = pressed;
                break;

            case RightBumper:
                if (pressed)
                {
                    driveSpeedScale = RobotParams.Robot.DRIVE_SLOW_SCALE;
                    turnSpeedScale = RobotParams.Robot.TURN_SLOW_SCALE;
                }
                else
                {
                    driveSpeedScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
                    turnSpeedScale = RobotParams.Robot.TURN_NORMAL_SCALE;
                }
                break;

            case DpadUp:
                break;
            case DpadDown:         
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
                }
                break;

            case Start:
                if (driverAltFunc)
                {
                    if (pressed)
                    {
                        subsystemStatusOn = !subsystemStatusOn;
                    }
                }
                else
                {
                    if (robot.photonVisionBack != null &&
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
     * This method is called when a driver joystick button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverJoystickButtonEvent(FrcJoystick.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            8, "DriveJoystick: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            default:
                break;
        }
    }   //driverJoystickButtonEvent

    /**
     * This method is called when a driver dual joystick button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void driverDualJoystickButtonEvent(FrcDualJoystick.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            8, "DriveDualJoystick: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            default:
                break;
        }
    }   //driverDualJoystickButtonEvent

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
            8, "OperatorController: " + button + "=" + (pressed ? "pressed" : "released"));

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
                        }
                        else
                        {
                            robot.coralGrabber.autoIntake(null);
                        }
                    }
                    else if (robot.coralGrabber.isAutoActive())
                    {
                        robot.coralGrabber.cancel();
                    }
                    else
                    {
                        robot.coralGrabber.stop();
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
                        }
                        else
                        {
                            robot.coralGrabber.autoEject(null, 1.0, null, 0.0);
                        }
                    }
                    else if (robot.coralGrabber.isAutoActive())
                    {
                        robot.coralGrabber.cancel();
                    }
                    else
                    {
                        robot.coralGrabber.stop();
                    }
                }
                break;

            case X:
                // Bindings for testing presets
                if(robot.coralArm != null && robot.elevator != null && pressed){
                    robot.coralArm.setPosition(CoralArm.Params.SCORE_LEVEL_POS[scoreIndex], true);
                    robot.elevator.setPosition(Elevator.Params.SCORE_LEVEL_POS[scoreIndex], true);
                }
                break;
            case Y:
                // Binding for testing presets
                if(robot.coralArm != null && pressed){
                    //robot.elevator.setPosition(Elevator.Params.STATION_PICKUP_POS, true);
                    robot.coralArm.setPosition(
                        moduleName, 0.0, CoralArm.Params.STATION_PICKUP_POS, true, CoralArm.Params.POWER_LIMIT, null,
                        0.0);
                }
                break;

            case LeftBumper:
                operatorAltFunc = pressed;
                break;

            case RightBumper:
                break;

            case DpadUp:

                // if (operatorAltFunc)
                // {
                //     if (robot.elevator != null)
                //     {
                //         if (pressed)
                //         {
                //             robot.elevator.presetPositionUp(moduleName, Elevator.Params.POWER_LIMIT);
                //         }
                //     }
                // }
                // else
                // {
                //     if (robot.coralArm != null)
                //     {
                //         if (pressed)
                //         {
                //             robot.coralArm.presetPositionUp(moduleName, CoralArm.Params.POWER_LIMIT);
                //         }
        
                //     }
                // }
                if(pressed && scoreIndex != 2){
                    scoreIndex++;
                }
            
                break;

            case DpadDown:
                // if (operatorAltFunc)
                // {
                //     if (robot.elevator != null)
                //     {
                //         if (pressed)
                //         {
                //             robot.elevator.presetPositionDown(moduleName, Elevator.Params.POWER_LIMIT);
                //         }
                //     }
                // }
                // else
                // {
                //     if (robot.coralArm != null)
                //     {
                //         if (pressed)
                //         {
                //             robot.coralArm.presetPositionDown(moduleName, CoralArm.Params.POWER_LIMIT);
                //         }
                //     }
                // }
                if(pressed && scoreIndex != 1){
                    scoreIndex--;
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
                }
                break;

            case Start:
                if (pressed)
                {
                    robot.cancelAll();
                }
                break;

            default:
                break;
        }
    }   //operatorControllerButtonEvent

    /**
     * This method is called when an operator stick button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void operatorStickButtonEvent(FrcJoystick.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            8, "OperatorStick: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            default:
                break;
        }
    }   //operatorStickButtonEvent

    /**
     * This method is called when a button panel button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void buttonPanelButtonEvent(FrcButtonPanel.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            8, "ButtonPanel: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            default:
                break;
        }
    }   //buttonPanelButtonEvent

    /**
     * This method is called when a switch panel button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    protected void switchPanelButtonEvent(FrcButtonPanel.ButtonType button, boolean pressed)
    {
        if (traceButtonEvents)
        {
            robot.globalTracer.traceInfo(moduleName, "##### button=" + button + ", pressed=" + pressed);
        }

        robot.dashboard.displayPrintf(
            8, "SwitchPanel: " + button + "=" + (pressed ? "pressed" : "released"));

        switch (button)
        {
            default:
                break;
        }
    }   //switchPanelButtonEvent

}   //class FrcTeleOp
