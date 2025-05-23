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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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

import java.util.Locale;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frclib.drivebase.FrcRobotDrive;
import frclib.drivebase.FrcSwerveDrive;
import frclib.driverio.FrcChoiceMenu;
import frclib.driverio.FrcUserChoices;
import frclib.driverio.FrcXboxController;
import frclib.vision.FrcPhotonVision;
import teamcode.subsystems.CoralArm;
import teamcode.subsystems.Elevator;
import teamcode.vision.PhotonVision.PipelineType;
import trclib.command.CmdDriveMotorsTest;
import trclib.command.CmdPidDrive;
import trclib.command.CmdTimedDrive;
import trclib.controller.TrcPidController;
import trclib.dataprocessor.TrcUtil;
import trclib.motor.TrcMotor;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcRobot;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.timer.TrcTimer;

/**
 * This class implements the code to run in Test Mode.
 */
public class FrcTest extends FrcTeleOp
{
    private static final String moduleName = FrcTest.class.getSimpleName();
    // Smart dashboard keys for Autonomous choices.
    private static final String DBKEY_TEST_TESTS = "Test/Tests";
    private static final String DBKEY_TEST_X_DRIVE_DISTANCE = "Test/XDriveDistance";
    private static final String DBKEY_TEST_Y_DRIVE_DISTANCE = "Test/YDriveDistance";
    private static final String DBKEY_TEST_TURN_ANGLE = "Test/TurnAngle";
    private static final String DBKEY_TEST_DRIVE_TIME = "Test/DriveTime";
    private static final String DBKEY_TEST_DRIVE_POWER = "Test/DrivePower";
    private static final String DBKEY_TEST_TUNE_KP = "Test/TuneKp";
    private static final String DBKEY_TEST_TUNE_KI = "Test/TuneKi";
    private static final String DBKEY_TEST_TUNE_KD = "Test/TuneKd";
    private static final String DBKEY_TEST_TUNE_KF = "Test/TuneKf";
    private static final String DBKEY_TEST_TUNE_IZONE = "Test/TuneIZone";
    private static final String DBKEY_TEST_TARGET_VEL = "Test/TargetVelocity";
    private static final String DBKEY_TEST_ROBOT_VEL = "Test/RobotVelocity";
    //
    // Global constants.
    //

    //
    // Tests.
    //
    private enum Test
    {
        SENSORS_TEST,
        SUBSYSTEMS_TEST,
        VISION_TEST,
        SWERVE_CALIBRATION,
        DRIVE_SPEED_TEST,
        DRIVE_MOTORS_TEST,
        X_TIMED_DRIVE,
        Y_TIMED_DRIVE,
        PP_DRIVE,
        PID_DRIVE,
        TUNE_PP_PID,
        TUNE_X_PID,
        TUNE_Y_PID,
        TUNE_TURN_PID,
        LIVE_WINDOW
    }   //enum Test

    /**
     * This class encapsulates all user choices for test mode from the smart dashboard.
     *
     * To add a test choice, follow the steps below:
     * 1. Add a DBKEY string constant.
     * 2. If the choice is a choice menu, create a FrcChoiceMenu variable for it, create the enum type if necessary,
     *    add code to create the FrcChoiceMenu object and add choices to it.
     * 3. Call userChoices to add the new choice object and provide default value if necessary.
     * 4. Add a getter method for the new choice.
     * 5. Add an entry of the new choice to the toString method.
     */
    class TestChoices
    {
        private final FrcUserChoices userChoices = new FrcUserChoices();
        private final FrcChoiceMenu<Test> testMenu;

        public TestChoices()
        {
            //
            // Create test mode specific choice menus.
            //
            testMenu = new FrcChoiceMenu<>(DBKEY_TEST_TESTS);
            //
            // Populate test mode menus.
            //
            testMenu.addChoice("Sensors Test", Test.SENSORS_TEST, true, false);
            testMenu.addChoice("Subsystems Test", Test.SUBSYSTEMS_TEST);
            testMenu.addChoice("Vision Test", Test.VISION_TEST);
            testMenu.addChoice("Swerve Calibration", Test.SWERVE_CALIBRATION);
            testMenu.addChoice("Drive Speed Test", Test.DRIVE_SPEED_TEST);
            testMenu.addChoice("Drive Motors Test", Test.DRIVE_MOTORS_TEST);
            testMenu.addChoice("X Timed Drive", Test.X_TIMED_DRIVE);
            testMenu.addChoice("Y Timed Drive", Test.Y_TIMED_DRIVE);
            testMenu.addChoice("PurePursuit Drive", Test.PP_DRIVE);
            testMenu.addChoice("PID Drive", Test.PID_DRIVE);
            testMenu.addChoice("Tune PurePursuit PID", Test.TUNE_PP_PID);
            testMenu.addChoice("Tune X PID", Test.TUNE_X_PID);
            testMenu.addChoice("Tune Y PID", Test.TUNE_Y_PID);
            testMenu.addChoice("Tune Turn PID", Test.TUNE_TURN_PID);
            testMenu.addChoice("Live Window", Test.LIVE_WINDOW, false, true);
            //
            // Initialize dashboard with default choice values.
            //
            userChoices.addChoiceMenu(DBKEY_TEST_TESTS, testMenu);
            userChoices.addNumber(DBKEY_TEST_X_DRIVE_DISTANCE, 0.0);    // in ft
            userChoices.addNumber(DBKEY_TEST_Y_DRIVE_DISTANCE, 0.0);    // in ft
            userChoices.addNumber(DBKEY_TEST_TURN_ANGLE, 0.0);          // in degrees
            userChoices.addNumber(DBKEY_TEST_DRIVE_TIME, 0.0);          // in seconds
            userChoices.addNumber(DBKEY_TEST_DRIVE_POWER, 1.0);
            userChoices.addNumber(DBKEY_TEST_TUNE_KP, 0.0);
            userChoices.addNumber(DBKEY_TEST_TUNE_KI, 0.0);
            userChoices.addNumber(DBKEY_TEST_TUNE_KD, 0.0);
            userChoices.addNumber(DBKEY_TEST_TUNE_KF, 0.0);
            userChoices.addNumber(DBKEY_TEST_TUNE_IZONE, 0.0);
            userChoices.addNumber(DBKEY_TEST_TARGET_VEL, 0.0);
            userChoices.addNumber(DBKEY_TEST_ROBOT_VEL, 0.0);
        }   //TestChoices

        //
        // Getters for test mode choices.
        //

        public Test getTest()
        {
            return testMenu.getCurrentChoiceObject();            
        }   //getTest

        public double getXDriveDistance()
        {
            return userChoices.getUserNumber(DBKEY_TEST_X_DRIVE_DISTANCE);
        }   //getXDriveDistance

        public double getYDriveDistance()
        {
            return userChoices.getUserNumber(DBKEY_TEST_Y_DRIVE_DISTANCE);
        }   //getYDriveDistance

        public double getTurnAngle()
        {
            return userChoices.getUserNumber(DBKEY_TEST_TURN_ANGLE);
        }   //getTurnAngle

        public double getDriveTime()
        {
            return userChoices.getUserNumber(DBKEY_TEST_DRIVE_TIME);
        }   //getDriveTime

        public double getDrivePower()
        {
            return userChoices.getUserNumber(DBKEY_TEST_DRIVE_POWER);
        }   //getDrivePower

        public TrcPidController.PidCoefficients getTunePidCoefficients()
        {
            return new TrcPidController.PidCoefficients(
                userChoices.getUserNumber(DBKEY_TEST_TUNE_KP),
                userChoices.getUserNumber(DBKEY_TEST_TUNE_KI),
                userChoices.getUserNumber(DBKEY_TEST_TUNE_KD),
                userChoices.getUserNumber(DBKEY_TEST_TUNE_KF),
                userChoices.getUserNumber(DBKEY_TEST_TUNE_IZONE));
        }   //getTunePidCoefficients

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "Test=\"%s\" " +
                "xDistance=\"%.1f ft\" " +
                "yDistance=\"%.1f ft\" " +
                "turnDegrees=\"%.0f deg\" " +
                "driveTime=\"%.0f sec\" " +
                "drivePower=\"%.1f\" " +
                "tunePidCoeff=\"%s\" ",
                getTest(), getXDriveDistance(), getYDriveDistance(), getTurnAngle(), getDriveTime(), getDrivePower(),
                getTunePidCoefficients());
        }   //toString

    }   //class TestChocies

    //
    // Global objects.
    //
    private final TestChoices testChoices = new TestChoices();
    private TrcRobot.RobotCommand testCommand;
    private double maxDriveVelocity = 0.0;
    private double maxDriveAcceleration = 0.0;
    private double maxDriveDeceleration = 0.0;
    private double maxTurnVelocity = 0.0;
    private double prevTime = 0.0;
    private double prevVelocity = 0.0;
    private PipelineType frontPipeline = PipelineType.APRILTAG;
    private PipelineType backPipeline = PipelineType.APRILTAG;

    public FrcTest(Robot robot)
    {
        //
        // Call TeleOp constructor.
        //
        super(robot);
        //
        // Create and initialize global objects.
        //

    }   //FrcTest

    //
    // Overriding TrcRobot.RobotMode.
    //

    @Override
    public void startMode(RunMode prevMode, RunMode nextMode)
    {
        if (RobotParams.Preferences.hybridMode)
        {
            // Cancels all running commands at the start of test mode.
            CommandScheduler.getInstance().cancelAll();
        }
        //
        // Call TeleOp startMode.
        //
        super.startMode(prevMode, nextMode);
        //
        // Retrieve Test choices.
        //
        robot.globalTracer.logInfo(moduleName, "TestChoices", "%s", testChoices);
        //
        // Create Command objects according to test choice.
        //
        boolean liveWindowEnabled = false;

        switch (testChoices.getTest())
        {
            case SENSORS_TEST:
                // Make sure no joystick controls on sensors test.
                setControlsEnabled(false);
                // Sensors Test is the same as Subsystems Test without joystick control.
                // So let it flow to the next case.
            case SUBSYSTEMS_TEST:
                break;

            case VISION_TEST:
                if (robot.photonVisionFront != null)
                {
                    robot.photonVisionFront.setPipeline(frontPipeline);
                }

                if (robot.photonVisionBack != null)
                {
                    robot.photonVisionBack.setPipeline(backPipeline);
                }
                break;

            case SWERVE_CALIBRATION:
                if (robot.robotDrive != null && robot.robotDrive instanceof FrcSwerveDrive)
                {
                    setControlsEnabled(false);
                    ((FrcSwerveDrive) robot.robotDrive).startSteeringCalibration();
                }
                break;

            case DRIVE_MOTORS_TEST:
                if (robot.robotDrive != null)
                {
                    // Initialize motor array with the wheel motors. For 2-motor drive base, it is leftWheel and
                    // rightWheel. For 4-motor drive base, it is lfWheel, rfWheel, lbWheel, rbWheel.
                    testCommand = new CmdDriveMotorsTest(
                        robot.robotDrive.driveBase, robot.robotDrive.driveMotors, 5.0, 0.5);
                }
                break;

            case X_TIMED_DRIVE:
                if (robot.robotDrive != null && robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    for (TrcMotor motor: robot.robotDrive.driveMotors)
                    {
                        motor.resetMotorPosition();
                    }
                    robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.getDriveTime(), testChoices.getDrivePower(),
                        0.0, 0.0);
                }
                break;

            case Y_TIMED_DRIVE:
                if (robot.robotDrive != null)
                {
                    for (TrcMotor motor: robot.robotDrive.driveMotors)
                    {
                        motor.resetMotorPosition();
                    }
                    robot.robotDrive.driveBase.setGyroAssistEnabled(robot.robotDrive.pidDrive.getTurnPidCtrl());
                    testCommand = new CmdTimedDrive(
                        robot.robotDrive.driveBase, 0.0, testChoices.getDriveTime(), 0.0, testChoices.getDrivePower(),
                        0.0);
                }
                break;

            case PP_DRIVE:
                if (robot.robotDrive != null)
                {
                    robot.robotDrive.purePursuitDrive.setMoveOutputLimit(testChoices.getDrivePower());
                    robot.robotDrive.purePursuitDrive.start(
                        null, true,
                        robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration,
                        robot.robotInfo.profiledMaxDeceleration,
                        new TrcPose2D(
                            testChoices.getXDriveDistance()*12.0, testChoices.getYDriveDistance()*12.0,
                            testChoices.getTurnAngle()));
                }
                break;

            case PID_DRIVE:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdPidDrive(robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.getDrivePower(), null,
                        new TrcPose2D(
                            testChoices.getXDriveDistance()*12.0, testChoices.getYDriveDistance()*12.0,
                            testChoices.getTurnAngle()));
                }
                break;

            case TUNE_PP_PID:
                if (robot.robotDrive != null && robot.robotDrive.purePursuitDrive != null)
                {
                    robot.robotDrive.driveBase.setFieldPosition(new TrcPose2D(0,0,0));
                    TrcPidController.PidCoefficients tunePidCoeff = testChoices.getTunePidCoefficients();
                    double xTarget = testChoices.getXDriveDistance()*12.0;
                    double yTarget = testChoices.getYDriveDistance()*12.0;
                    double turnTarget = testChoices.getTurnAngle();
                    double drivePower = testChoices.getDrivePower();
                    if (turnTarget != 0.0)
                    {
                        robot.robotDrive.purePursuitDrive.setTurnPidCoefficients(tunePidCoeff);
                        robot.robotDrive.purePursuitDrive.setRotOutputLimit(drivePower);
                    }
                    else
                    {
                        robot.robotDrive.purePursuitDrive.setPositionPidCoefficients(tunePidCoeff);
                        robot.robotDrive.purePursuitDrive.setMoveOutputLimit(drivePower);
                    }
                    robot.robotDrive.purePursuitDrive.start(
                        null, true,
                        robot.robotInfo.profiledMaxVelocity,
                        robot.robotInfo.profiledMaxAcceleration,
                        robot.robotInfo.profiledMaxDeceleration,
                        new TrcPose2D(xTarget, yTarget, turnTarget));
                }
                break;

            case TUNE_X_PID:
                if (robot.robotDrive != null && robot.robotDrive.driveBase.supportsHolonomicDrive())
                {
                    testCommand = new CmdPidDrive(robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.getDrivePower(), testChoices.getTunePidCoefficients(),
                        new TrcPose2D(testChoices.getXDriveDistance()*12.0, 0.0, 0.0));
                }
                break;

            case TUNE_Y_PID:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdPidDrive(robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.getDrivePower(), testChoices.getTunePidCoefficients(),
                        new TrcPose2D(0.0, testChoices.getYDriveDistance()*12.0, 0.0));
                }
                break;

            case TUNE_TURN_PID:
                if (robot.robotDrive != null)
                {
                    testCommand = new CmdPidDrive(robot.robotDrive.driveBase, robot.robotDrive.pidDrive);
                    ((CmdPidDrive) testCommand).start(
                        0.0, testChoices.getDrivePower(), testChoices.getTunePidCoefficients(),
                        new TrcPose2D(0.0, 0.0, testChoices.getTurnAngle()));
                }
                break;

            case LIVE_WINDOW:
                liveWindowEnabled = true;
                break;

            default:
                break;
        }

        LiveWindow.setEnabled(liveWindowEnabled);
        //
        // Start test state machine if necessary.
        //

    }   //startMode

    @Override
    public void stopMode(RunMode prevMode, RunMode nextMode)
    {
        super.stopMode(prevMode, nextMode);
        switch (testChoices.getTest())
        {
            case SWERVE_CALIBRATION:
                if (robot.robotDrive != null && robot.robotDrive instanceof FrcSwerveDrive)
                {
                    ((FrcSwerveDrive) robot.robotDrive).stopSteeringCalibration();
                }
                break;

            case X_TIMED_DRIVE:
            case Y_TIMED_DRIVE:
                robot.robotDrive.driveBase.setGyroAssistEnabled(null);
                break;

            default:
                break;
        }
    }   //stopMode

    //
    // Must override TeleOp so it doesn't fight with us.
    //

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

        if (testCommand != null)
        {
            testCommand.cmdPeriodic(elapsedTime);
        }
        //
        // Run test Cmd.
        //
        switch (testChoices.getTest())
        {
            case SENSORS_TEST:
                super.periodic(elapsedTime, slowPeriodicLoop);
                break;

            case DRIVE_SPEED_TEST:
                if (robot.robotDrive != null)
                {
                    double currTime = TrcTimer.getCurrentTime();
                    TrcPose2D velPose = robot.robotDrive.driveBase.getFieldVelocity();
                    double velocity = TrcUtil.magnitude(velPose.x, velPose.y);
                    double acceleration = 0.0;
                    double deceleration = 0.0;
                    double deltaTime = currTime - prevTime;

                    if (prevTime != 0.0)
                    {
                        if (velocity > prevVelocity)
                        {
                            acceleration = (velocity - prevVelocity)/deltaTime;
                        }
                        else
                        {
                            deceleration = (prevVelocity - velocity)/deltaTime;
                        }
                    }

                    if (velocity > maxDriveVelocity)
                    {
                        maxDriveVelocity = velocity;
                    }

                    if (acceleration > maxDriveAcceleration)
                    {
                        maxDriveAcceleration = acceleration;
                    }

                    if (deceleration > maxDriveDeceleration)
                    {
                        maxDriveDeceleration = deceleration;
                    }

                    if (velPose.angle > maxTurnVelocity)
                    {
                        maxTurnVelocity = velPose.angle;
                    }

                    prevTime = currTime;
                    prevVelocity = velocity;

                    robot.dashboard.displayPrintf(lineNum++, "Drive Vel: (%.1f/%.1f)", velocity, maxDriveVelocity);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Accel: (%.1f/%.1f)", acceleration, maxDriveAcceleration);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Drive Decel: (%.1f/%.1f)", deceleration, maxDriveDeceleration);
                    robot.dashboard.displayPrintf(
                        lineNum++, "Turn Vel: (%.1f/%.1f)", velPose.angle, maxTurnVelocity);
                }
                break;

            case PP_DRIVE:
                robot.dashboard.putNumber(
                    DBKEY_TEST_TARGET_VEL, robot.robotDrive.purePursuitDrive.getPathTargetVelocity());
                robot.dashboard.putNumber(
                    DBKEY_TEST_ROBOT_VEL, robot.robotDrive.purePursuitDrive.getPathRobotVelocity());
                break;

            default:
                break;
        }

        if (slowPeriodicLoop)
        {
            if (allowTeleOp())
            {
                //
                // Allow TeleOp to run so we can control the robot in subsystem test or drive speed test modes.
                //
                super.periodic(elapsedTime, true);
            }
            //
            // Call super.runPeriodic only if you need TeleOp control of the robot.
            //
            switch (testChoices.getTest())
            {
                case SENSORS_TEST:
                case SUBSYSTEMS_TEST:
                case VISION_TEST:
                    displaySensorStates(lineNum);
                    break;

                case SWERVE_CALIBRATION:
                    if (robot.robotDrive != null && robot.robotDrive instanceof FrcSwerveDrive)
                    {
                        FrcSwerveDrive swerveDrive = (FrcSwerveDrive) robot.robotDrive;
                        swerveDrive.runSteeringCalibration();
                        swerveDrive.displaySteerZeroCalibration(lineNum);
                    }
                    break;

                case X_TIMED_DRIVE:
                case Y_TIMED_DRIVE:
                    if (robot.robotDrive != null)
                    {
                        double lfEnc = Math.abs(
                            robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getMotorPosition());
                        double rfEnc = Math.abs(
                            robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getMotorPosition());
                        double lbEnc = Math.abs(
                            robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK] != null?
                                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK].getMotorPosition(): 0.0);
                        double rbEnc = Math.abs(
                            robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK] != null?
                                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getMotorPosition(): 0.0);
                        robot.dashboard.displayPrintf(lineNum++, "Enc:lf=%f,rf=%f", lfEnc, rfEnc);
                        robot.dashboard.displayPrintf(lineNum++, "Enc:lb=%f,rb=%f", lbEnc, rbEnc);
                        robot.dashboard.displayPrintf(lineNum++, "EncAverage=%f", (lfEnc + rfEnc + lbEnc + rbEnc) / 4.0);
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                    }
                    break;

                case PP_DRIVE:
                case PID_DRIVE:
                case TUNE_PP_PID:
                case TUNE_X_PID:
                case TUNE_Y_PID:
                case TUNE_TURN_PID:
                    if (robot.robotDrive != null)
                    {
                        robot.dashboard.displayPrintf(
                            lineNum++, "RobotPose=%s", robot.robotDrive.driveBase.getFieldPosition());
                        TrcPidController xPidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                        if (xPidCtrl != null)
                        {
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        robot.robotDrive.pidDrive.getYPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robot.robotDrive.pidDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                    }
                    break;

                default:
                    break;
            }
            //
            // Update Dashboard.
            //
            Dashboard.updateDashboard(robot, lineNum);
        }
    }   //periodic

    /**
     * This method is called to determine if Test mode is allowed to do teleop control of the robot.
     *
     * @return true to allow and false otherwise.
     */
    private boolean allowTeleOp()
    {
        Test test = testChoices.getTest();

        return test == Test.SUBSYSTEMS_TEST || test == Test.VISION_TEST || test == Test.DRIVE_SPEED_TEST;
    }   //allowTeleOp

    //
    // Overriding ButtonEvent here if necessary.
    //
    /**
     * This method is called when an operator controller button event is detected.
     *
     * @param button specifies the button that generated the event.
     * @param pressed specifies true if the button is pressed, false otherwise.
     */
    @Override
    protected void operatorControllerButtonEvent(FrcXboxController.ButtonType button, boolean pressed)
    {
        boolean passToTeleOp = true;

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
            case LeftBumper:
            case RightBumper:
                break;

            case DpadUp:
                if (robot.elevatorArmTask != null)
                {
                    if (operatorAltFunc)
                    {
                        if (robot.elevatorArmTask.elevator != null && pressed)
                        {
                            robot.elevatorArmTask.elevator.presetPositionUp(
                                moduleName, Elevator.Params.POWER_LIMIT);
                        }
                    }
                    else
                    {
                        if (robot.elevatorArmTask.coralArm != null && pressed)
                        {
                            robot.elevatorArmTask.coralArm.presetPositionUp(
                                moduleName, CoralArm.Params.POWER_LIMIT);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadDown:
                if (robot.elevatorArmTask != null)
                {
                    if (operatorAltFunc)
                    {
                        if (robot.elevatorArmTask.elevator != null && pressed)
                        {
                            robot.elevatorArmTask.elevator.presetPositionDown(
                                moduleName, Elevator.Params.POWER_LIMIT);
                        }
                    }
                    else
                    {
                        if (robot.elevatorArmTask.coralArm != null && pressed)
                        {
                            robot.elevatorArmTask.coralArm.presetPositionDown(
                                moduleName, CoralArm.Params.POWER_LIMIT);
                        }
                    }
                    passToTeleOp = false;
                }
                break;

            case DpadLeft:
            case DpadRight:
            case Back:
            case Start:
            default:
                break;
        }
        //
        // If the control was not processed by this method, pass it back to TeleOp.
        //
        if (passToTeleOp)
        {
            super.operatorControllerButtonEvent(button, pressed);
        }
    }   //operatorControllerButtonEvent

    //
    // Implement tests.
    //

    /**
     * This method reads all sensors and prints out their values. This is a very
     * useful diagnostic tool to check if all sensors are working properly. For
     * encoders, since test subsystem mode is also teleop mode, you can operate
     * the joysticks to turn the motors and check the corresponding encoder
     * counts.
     *
     * @param lineNum specifies the dashboard starting line number to display the info.
     */
    private void displaySensorStates(int lineNum)
    {
        //
        // Display drivebase info.
        //
        if (robot.battery != null)
        {
            robot.dashboard.displayPrintf(
                lineNum++, "Sensors Test (Batt=%.1f/%.1f):", robot.battery.getVoltage(), robot.battery.getLowestVoltage());
        }

        if (robot.robotDrive != null)
        {
            robot.dashboard.displayPrintf(
                lineNum++, "DriveBase: Pose=%s,Vel=%s", robot.robotDrive.driveBase.getFieldPosition(),
                robot.robotDrive.driveBase.getFieldVelocity());
            robot.dashboard.displayPrintf(lineNum++, "DrivePower: lf=%.2f,rf=%.2f,lb=%.2f,rb=%.2f",
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getMotorPower(),
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getMotorPower(),
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK] != null?
                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK].getMotorPower(): 0.0,
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK] != null?
                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getMotorPower(): 0.0);
            robot.dashboard.displayPrintf(lineNum++, "DriveEnc: lf=%.1f,rf=%.1f,lb=%.1f,rb=%.1f",
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getPosition(),
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getPosition(),
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK] != null?
                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK].getPosition(): 0.0,
                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK] != null?
                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getPosition(): 0.0);
            if (robot.robotDrive instanceof FrcSwerveDrive)
            {
                FrcSwerveDrive swerveDrive = (FrcSwerveDrive) robot.robotDrive;
                robot.dashboard.displayPrintf(lineNum++, "SteerEnc: lf=%f, rf=%f, lb=%f, rb=%f",
                    swerveDrive.steerEncoders[FrcRobotDrive.INDEX_LEFT_FRONT].getRawPosition(),
                    swerveDrive.steerEncoders[FrcRobotDrive.INDEX_RIGHT_FRONT].getRawPosition(),
                    swerveDrive.steerEncoders[FrcRobotDrive.INDEX_LEFT_BACK].getRawPosition(),
                    swerveDrive.steerEncoders[FrcRobotDrive.INDEX_RIGHT_BACK].getRawPosition());
            }
        }

        if (robot.photonVisionFront != null)
        {
            FrcPhotonVision.DetectedObject object = robot.photonVisionFront.getBestDetectedObject(null);
            if (object != null)
            {
                robot.dashboard.displayPrintf(
                    lineNum++, "PhotonFront: pipeline=%s, pose=%s", frontPipeline, object.targetPose);
            }
            else
            {
                lineNum++;
            }
        }

        if (robot.photonVisionBack != null)
        {
            FrcPhotonVision.DetectedObject object = robot.photonVisionBack.getBestDetectedObject(null);
            if (object != null)
            {
                robot.dashboard.displayPrintf(
                    lineNum++, "PhotonBack: pipeline=%s, pose=%s", backPipeline, object.targetPose);
            }
            else
            {
                lineNum++;
            }
        }
        //
        // Display other subsystems and sensor info.
        //

    }   //displaySensorStates

}   //class FrcTest
