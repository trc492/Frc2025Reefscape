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

package teamcode;

import frclib.driverio.FrcButtonPanel;
import frclib.driverio.FrcDualJoystick;
import frclib.driverio.FrcJoystick;
import frclib.driverio.FrcXboxController;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;

/**
 * This class implements the code to run in TeleOp Mode.
 */
public class FrcTeleOp implements TrcRobot.RobotMode
{
    private static final String moduleName = FrcTeleOp.class.getSimpleName();
    private static final boolean traceButtonEvents = true;
    //
    // Global objects.
    //
    protected final Robot robot;
    private boolean controlsEnabled = false;
    private double driveSpeedScale = RobotParams.Robot.DRIVE_NORMAL_SCALE;
    private double turnSpeedScale = RobotParams.Robot.TURN_NORMAL_SCALE;
    protected boolean driverAltFunc = false;
    protected boolean operatorAltFunc = false;
    private boolean subsystemStatusOn = true;

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
                //
                // DriveBase operation.
                //
                if (robot.robotDrive != null)
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
                //
                // Analog control of subsystem is done here if necessary.
                //
                if (RobotParams.Preferences.useSubsystems)
                {
                }
            }
            //
            // Update robot status.
            //
            if (RobotParams.Preferences.doStatusUpdate)
            {
                robot.updateStatus(2);
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
                        robot.setDriveOrientation(DriveOrientation.FIELD, true);
                    }
                    else
                    {
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
            case Y:
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
            case DpadDown:
            case DpadLeft:
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
                    subsystemStatusOn = !subsystemStatusOn;
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
            case B:
            case X:
            case Y:
                break;

            case LeftBumper:
                operatorAltFunc = pressed;
                break;

            case RightBumper:
            case DpadUp:
            case DpadDown:
            case DpadLeft:
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
