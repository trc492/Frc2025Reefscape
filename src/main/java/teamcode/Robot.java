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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,g
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.PrintStream;
import java.util.Locale;
import java.util.Scanner;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frclib.drivebase.FrcRobotDrive;
import frclib.drivebase.FrcSwerveDrive;
import frclib.drivebase.FrcRobotDrive.ImuType;
import frclib.driverio.FrcButtonPanel;
import frclib.driverio.FrcDashboard;
import frclib.driverio.FrcDualJoystick;
import frclib.driverio.FrcJoystick;
import frclib.driverio.FrcMatchInfo;
import frclib.driverio.FrcXboxController;
import frclib.robotcore.FrcRobotBase;
import frclib.sensor.FrcAHRSGyro;
import frclib.sensor.FrcPdp;
import frclib.sensor.FrcRobotBattery;
import frclib.vision.FrcPhotonVision;
import teamcode.subsystems.LEDIndicator;
import teamcode.subsystems.RobotBase;
import teamcode.vision.OpenCvVision;
import teamcode.vision.PhotonVision;
import trclib.dataprocessor.TrcUtil;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcDbgTrace;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcPidController;
import trclib.robotcore.TrcRobot.RunMode;
import trclib.sensor.TrcRobotBattery;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcVisionTargetInfo;

/**
 * The Main class is configured to instantiate and automatically run this class,
 * and to call the functions corresponding to each mode, as described in the TrcRobot
 * documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main class to reflect the name change.
 */
public class Robot extends FrcRobotBase
{
    // Global objects.
    public static final String moduleName = Robot.class.getSimpleName();
    public final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    public final FrcDashboard dashboard = FrcDashboard.getInstance();
    private double nextDashboardUpdateTime = TrcTimer.getCurrentTime();
    private boolean traceLogOpened = false;
    // Inputs.
    public FrcXboxController driverController;
    public FrcJoystick driverJoystick;
    public FrcDualJoystick driverDualJoystick;
    public FrcXboxController operatorController;
    public FrcJoystick operatorStick;
    public FrcButtonPanel buttonPanel;
    public FrcButtonPanel switchPanel;
    // Sensors.
    public FrcPdp pdp;
    public TrcRobotBattery battery;
    public AnalogInput pressureSensor;
    // Miscellaneous hardware.
    public LEDIndicator ledIndicator;
    // Robot Drive.
    public RobotBase robotBase;
    public FrcRobotDrive.RobotInfo robotInfo;
    public FrcRobotDrive robotDrive;
    private TrcPose2D endOfAutoRobotPose = null;
    // Vision.
    public PhotonVision photonVisionFront;
    public PhotonVision photonVisionBack;
    public OpenCvVision openCvVision;
    // Hybrid mode objects.
    public Command m_autonomousCommand;
    //
    // Other subsystems.
    //

    //
    // Auto-Assists.
    //

    /**
     * Constructor: Create an instance of the object.
     */
    public Robot()
    {
        super(RobotParams.Robot.ROBOT_CODEBASE);
    }   //Robot

    /**
     * This method is called when the robot is first started up and should be used for any initialization code
     * including creation and initialization of all robot hardware and subsystems.
     *
     * To create new hardware or subsystem, follow the steps below:
     * 1. Create a public class variable for the new hardware/subsystem.
     * 2. Instantiate and initialize the new hardware/subsystem object in this method.
     * 3. Put code in updateStatus to display status of the new hardware/subsystem if necessary.
     * 4. Put code in robotStartMode or robotStopMode to configure/reset hardware/subsystem if necessary.
     * 5. Put code in FrcTeleOp to operate the subsystem if necessary (i.e. slowPeriodic/xxxButtonEvent).
     * 6. Create a getter method for the new sensor only if necessary (e.g. sensor value needs translation).
     */
    @Override
    public void robotInit()
    {
        // Enable LostComm detection.
        if (RobotParams.Preferences.useCommStatusMonitor)
        {
            super.setCommStatusMonitorEnabled(this::commStatusCallback);
        }

        // Create and initialize inputs.
        if (RobotParams.Preferences.useDriverXboxController)
        {
            driverController = new FrcXboxController("DriverController", RobotParams.HwConfig.XBOX_DRIVER_CONTROLLER);
            driverController.setLeftStickInverted(false, true);
            driverController.setRightStickInverted(false, true);
        }
        else if (RobotParams.Preferences.doOneStickDrive)
        {
            driverJoystick = new FrcJoystick("DriverJoystick", RobotParams.HwConfig.JSPORT_DRIVER_RIGHTSTICK);
            driverJoystick.setInverted(false, true);
        }
        else
        {
            driverDualJoystick = new FrcDualJoystick(
                "DriverDualJoystick", RobotParams.HwConfig.JSPORT_DRIVER_LEFTSTICK,
                RobotParams.HwConfig.JSPORT_DRIVER_RIGHTSTICK);
            driverDualJoystick.setLeftStickInverted(false, true);
        }

        if (RobotParams.Preferences.useOperatorXboxController)
        {
            operatorController = new FrcXboxController(
                "OperatorController", RobotParams.HwConfig.XBOX_OPERATOR_CONTROLLER);
            operatorController.setLeftStickInverted(false, true);
            operatorController.setRightStickInverted(false, true);
        }
        else
        {
            operatorStick = new FrcJoystick("operatorJoystick", RobotParams.HwConfig.JSPORT_OPERATORSTICK);
            operatorStick.setInverted(false, true);
        }

        if (RobotParams.Preferences.useButtonPanels)
        {
            buttonPanel = new FrcButtonPanel("buttonPanel", RobotParams.HwConfig.JSPORT_BUTTON_PANEL);
            switchPanel = new FrcButtonPanel("switchPanel", RobotParams.HwConfig.JSPORT_SWITCH_PANEL);
        }

        // Create and initialize sensors.
        if (RobotParams.Preferences.usePdp)
        {
            pdp = new FrcPdp(RobotParams.HwConfig.CANID_PDP, RobotParams.HwConfig.PDP_MODULE_TYPE);
            pdp.setSwitchableChannel(false);
            battery = new FrcRobotBattery(pdp);
        }

        if (RobotParams.Preferences.usePressureSensor)
        {
            pressureSensor = new AnalogInput(RobotParams.HwConfig.AIN_PRESSURE_SENSOR);
        }

        // Create and initialize miscellaneous hardware.
        ledIndicator = new LEDIndicator();

        // Create and initialize RobotInfo. This must be done early because subsequent components may require it.
        robotBase = new RobotBase();
        robotInfo = robotBase.getRobotInfo();
        robotDrive = robotBase.getRobotDrive();

        // Create and initialize Vision subsystem.
        if (RobotParams.Preferences.useVision)
        {
            if (RobotParams.Preferences.usePhotonVision)
            {
                photonVisionFront = robotInfo.cam1 != null?
                    new PhotonVision(robotInfo.cam1.camName, robotInfo.cam1.robotToCam, ledIndicator): null;
                photonVisionBack = robotInfo.cam2 != null?
                    new PhotonVision(robotInfo.cam2.camName, robotInfo.cam2.robotToCam, ledIndicator): null;
            }
            else if (RobotParams.Preferences.useOpenCvVision && robotInfo.cam2 != null)
            {
                UsbCamera camera = CameraServer.startAutomaticCapture(1);
                camera.setResolution(robotInfo.cam2.camImageWidth, robotInfo.cam2.camImageHeight);
                camera.setFPS(10);
                openCvVision = new OpenCvVision(
                    "OpenCvVision", 1, robotInfo.cam2,
                    CameraServer.getVideo(),
                    CameraServer.putVideo(
                        "UsbWebcam", robotInfo.cam2.camImageWidth, robotInfo.cam2.camImageHeight));
            }

            if (RobotParams.Preferences.useStreamCamera)
            {
                UsbCamera camera = CameraServer.startAutomaticCapture("DriverDisplay", 0);
                camera.setResolution(160, 120);
                camera.setFPS(10);
            }
        }

        //
        // Create and initialize other subsystems.
        //
        if (RobotParams.Preferences.useSubsystems)
        {
            // Create subsystems.

            // Zero calibrate all subsystems only in Auto or if TeleOp is run standalone without prior Auto.
            zeroCalibrate(null, null);

            // Create autotasks.
        }

        // Miscellaneous.
        if (pdp != null)
        {
            pdp.registerEnergyUsedForAllUnregisteredChannels();
        }
        //
        // Create Robot Modes.
        //
        setupRobotModes(new FrcTeleOp(this), new FrcAuto(this), new FrcTest(this), new FrcDisabled(this));
    }   //robotInit

    /**
     * This method is called to prepare the robot before a robot mode is about to start.
     *
     * @param runMode specifies the current run mode.
     * @param prevMode specifies the previous run mode.
     */
    @Override
    public void robotStartMode(RunMode runMode, RunMode prevMode)
    {
        cancelAll();
        // Read FMS Match info.
        FrcMatchInfo matchInfo = FrcMatchInfo.getMatchInfo();
        if (runMode != RunMode.DISABLED_MODE)
        {
            // Start trace logging.
            if (RobotParams.Preferences.useTraceLog)
            {
                openTraceLog(matchInfo);
                setTraceLogEnabled(true);
            }
            globalTracer.traceInfo(moduleName, "%s: ***** %s *****", matchInfo.eventDate, runMode);
            // Start RobotDrive.
            if (robotDrive != null)
            {
                robotDrive.driveBase.setOdometryEnabled(true, true);
                // Disable ramp rate control in autonomous.
                double rampTime = runMode == RunMode.AUTO_MODE? 0.0: RobotParams.Robot.DRIVE_RAMP_RATE;
                for (int i = 0; i < robotDrive.driveMotors.length; i++)
                {
                    robotDrive.driveMotors[i].setOpenLoopRampRate(rampTime);
                }

                if (runMode != RunMode.AUTO_MODE)
                {
                    if (runMode == RunMode.TELEOP_MODE && endOfAutoRobotPose != null)
                    {
                        robotDrive.driveBase.setFieldPosition(endOfAutoRobotPose);
                        endOfAutoRobotPose = null;
                    }

                    if (RobotParams.Preferences.useGyroAssist)
                    {
                        robotDrive.driveBase.setGyroAssistEnabled(robotDrive.pidDrive.getTurnPidCtrl());
                    }
                }
            }
            // Start subsystems.
            ledIndicator.reset();
        }
    }   //robotStartMode

    /**
     * This method is called to prepare the robot right after a robot mode has been stopped.
     *
     * @param runMode specifies the current run mode.
     * @param nextMode specifies the next run mode.
     */
    @Override
    public void robotStopMode(RunMode runMode, RunMode nextMode)
    {
        cancelAll();
        // Stop RobotDrive.
        if (runMode != RunMode.DISABLED_MODE && robotDrive != null)
        {
            robotDrive.cancel();

            if (runMode == RunMode.AUTO_MODE)
            {
                endOfAutoRobotPose = robotDrive.driveBase.getFieldPosition();
            }
            robotDrive.driveBase.setOdometryEnabled(false);
            robotDrive.pidDrive.pidDriveTaskProfiler.printPerformanceMetrics(robotDrive.pidDrive.tracer);
        }
        // Stop subsystems.
        ledIndicator.reset();
        // Performance status report.
        if (battery != null)
        {
            double totalEnergy = battery.getTotalEnergy();
            globalTracer.traceInfo(
                moduleName, "TotalEnergy=%.3fWh (%.2f%%)",
                totalEnergy, totalEnergy * 100.0 / RobotParams.HwConfig.BATTERY_CAPACITY_WATT_HOUR);
        }

        if (runMode != RunMode.DISABLED_MODE)
        {
            printPerformanceMetrics(globalTracer);
        }
        // Stop trace logging.
        setTraceLogEnabled(false);
        closeTraceLog();
    }   //robotStopMode

    /**
     * This method is called periodically in the specified run mode. This is typically used to execute periodic tasks
     * that's common to all run modes.
     *
     * @param runMode specifies the current run mode.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    @Override
    public void robotPeriodic(RunMode runMode, boolean slowPeriodicLoop)
    {
        if (RobotParams.Preferences.hybridMode)
        {
            // Runs the Command Based Scheduler. This is responsible for polling buttons, adding newly-scheduled
            // commands, running already-scheduled commands, removing finished or interrupted commands, and running
            // subsystem periodic() methods.  This must be called from the robot's periodic block in order for anything
            // in the Command-based framework to work.
            CommandScheduler.getInstance().run();
        }
    }   //robotPeriodic

    /**
     * This method is called periodically to update various hardware/subsystem status of the robot to the dashboard
     * and trace log. In order to lower the potential impact these updates, this method will only update the dashboard
     * at DASHBOARD_UPDATE_INTERVAL.
     *
     * @param lineNum specifies the first Dashboard line for printing status.
     */
    public void updateStatus(int lineNum)
    {
        double currTime = TrcTimer.getCurrentTime();
        RunMode runMode = getCurrentRunMode();

        if (currTime >= nextDashboardUpdateTime)
        {
            nextDashboardUpdateTime = currTime + RobotParams.Robot.DASHBOARD_UPDATE_INTERVAL;
            if (RobotParams.Preferences.showPowerConsumption)
            {
                if (pdp != null)
                {
                    dashboard.putNumber("Power/pdpTotalCurrent", pdp.getTotalCurrent());
                    dashboard.putNumber("Power/totalEnergy", battery.getTotalEnergy());
                    dashboard.putData("Power/pdpInfo", pdp.getPdpSendable());
                    if (runMode == RunMode.TELEOP_MODE)
                    {
                        globalTracer.traceInfo(
                            moduleName, "Battery: currVoltage=%.2f, lowestVoltage=%.2f",
                            battery.getVoltage(), battery.getLowestVoltage());
                        globalTracer.traceInfo(moduleName, "Total=%.2fA", pdp.getTotalCurrent());
                    }
                }
            }

            if (RobotParams.Preferences.showDriveBase)
            {
                if (robotDrive != null)
                {
                    TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
                    //
                    // DriveBase debug info.
                    //
                    double lfDriveEnc = robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getPosition();
                    double rfDriveEnc = robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getPosition();
                    double lbDriveEnc = robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK] != null?
                                            robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK].getPosition(): 0.0;
                    double rbDriveEnc = robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK] != null?
                                            robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getPosition(): 0.0;

                    dashboard.displayPrintf(
                        lineNum++, "DriveEnc: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                        lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                        (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) / 4.0);

                    if (robotDrive instanceof FrcSwerveDrive)
                    {
                        FrcSwerveDrive swerveDrive = (FrcSwerveDrive) robotDrive;

                        dashboard.displayPrintf(
                            lineNum++,
                            "FrontSteer(angle/motorEnc/absEnc): lf=%.1f/%.3f/%.3f, rf=%.1f/%.3f/%.3f",
                            swerveDrive.swerveModules[FrcRobotDrive.INDEX_LEFT_FRONT].getSteerAngle(),
                            swerveDrive.steerMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getMotorPosition(),
                            swerveDrive.steerEncoders[FrcRobotDrive.INDEX_LEFT_FRONT].getRawPosition(),
                            swerveDrive.swerveModules[FrcRobotDrive.INDEX_RIGHT_FRONT].getSteerAngle(),
                            swerveDrive.steerMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getMotorPosition(),
                            swerveDrive.steerEncoders[FrcRobotDrive.INDEX_RIGHT_FRONT].getRawPosition());
                        dashboard.displayPrintf(
                            lineNum++,
                            "BackSteer(angle/motorEnc/absEnc): lb=%.1f/%.3f/%.3f, rb=%.1f/%.3f/%.3f",
                            swerveDrive.swerveModules[FrcRobotDrive.INDEX_LEFT_BACK].getSteerAngle(),
                            swerveDrive.steerMotors[FrcRobotDrive.INDEX_LEFT_BACK].getMotorPosition(),
                            swerveDrive.steerEncoders[FrcRobotDrive.INDEX_LEFT_BACK].getRawPosition(),
                            swerveDrive.swerveModules[FrcRobotDrive.INDEX_RIGHT_BACK].getSteerAngle(),
                            swerveDrive.steerMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getMotorPosition(),
                            swerveDrive.steerEncoders[FrcRobotDrive.INDEX_RIGHT_BACK].getRawPosition());
                    }
                    dashboard.displayPrintf(lineNum++, "DriveBase: pose=%s", robotPose);

                    if (RobotParams.Preferences.showPidDrive)
                    {
                        TrcPidController xPidCtrl = robotDrive.pidDrive.getXPidCtrl();
                        if (xPidCtrl != null)
                        {
                            xPidCtrl.displayPidInfo(lineNum);
                            lineNum += 2;
                        }
                        robotDrive.pidDrive.getYPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                        robotDrive.pidDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                        lineNum += 2;
                    }
                }
            }

            if (RobotParams.Preferences.showVision)
            {
                if (photonVisionFront != null)
                {
                    FrcPhotonVision.DetectedObject object = photonVisionFront.getBestDetectedObject();
                    if (object != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "PhotonFront: pipeline=%s, obj=%s", photonVisionFront.getPipeline(), object);
                    }
                    else
                    {
                        lineNum++;
                    }
                }

                if (photonVisionBack != null)
                {
                    FrcPhotonVision.DetectedObject object = photonVisionBack.getBestDetectedObject();
                    if (object != null)
                    {
                        dashboard.displayPrintf(
                            lineNum++, "PhotonBack: pipeline=%s, obj=%s", photonVisionBack.getPipeline(), object);
                    }
                    else
                    {
                        lineNum++;
                    }
                }

                if (openCvVision != null)
                {
                    TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> object =
                        openCvVision.getDetectedTargetInfo(null, null);
                    if (object != null)
                    {
                        dashboard.displayPrintf(lineNum++, "OpenCv: %s", object);
                    }
                    else
                    {
                        lineNum++;
                    }
                }
            }
            //
            // Display other subsystem status here.
            //
            if (RobotParams.Preferences.showSubsystems)
            {
            }
        }
    }   //updateStatus

    /**
     * This method is called to cancel all pending operations and release the ownership of all subsystems.
     */
    public void cancelAll()
    {
        globalTracer.traceInfo(moduleName, "Cancel all operations.");
        // Cancel subsystems.
        if (robotDrive != null) robotDrive.cancel();
        // Cancel auto tasks.
    }   //cancelAll

    /**
     * This method zero calibrates all subsystems.
     *
     * @param owner specifies the owner ID to check if the caller has ownership of the motor.
     * @param event specifies the event to signal when the zero calibration is done.
     */
    public void zeroCalibrate(String owner, TrcEvent event)
    {
    }   //zeroCalibrate

    /**
     * This method retracts all appendages for robot high speed travelling.
     */
    public void turtle()
    {
    }   //turtle

    /**
     * This method creates and opens the trace log with the file name derived from the given match info.
     * Note that the trace log is disabled after it is opened. The caller must explicitly call setTraceLogEnabled
     * to enable/disable it.
     *
     * @param matchInfo specifies the match info from which the trace log file name is derived.
     */
    public void openTraceLog(FrcMatchInfo matchInfo)
    {
        if (RobotParams.Preferences.useTraceLog && !traceLogOpened)
        {
            String fileName = matchInfo.eventName != null?
                String.format(Locale.US, "%s_%s%03d", matchInfo.eventName, matchInfo.matchType, matchInfo.matchNumber):
                getCurrentRunMode().name();

            traceLogOpened = TrcDbgTrace.openTraceLog(RobotParams.Robot.LOG_FOLDER_PATH, fileName);
        }
    }   //openTraceLog

    /**
     * This method closes the trace log if it was opened.
     */
    public void closeTraceLog()
    {
        if (traceLogOpened)
        {
            TrcDbgTrace.closeTraceLog();
            traceLogOpened = false;
        }
    }   //closeTraceLog

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false to disable.
     */
    public void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogOpened)
        {
            TrcDbgTrace.setTraceLogEnabled(enabled);
        }
    }   //setTraceLogEnabled

    /**
     * This method retrieves the field zero compass heading from the calibration data file.
     *
     * @return calibration data of field zero compass heading.
     */
    private Double getFieldZeroCompassHeading()
    {
        try (Scanner in = new Scanner(new FileReader(RobotParams.Robot.FIELD_ZERO_CAL_FILE)))
        {
            return in.nextDouble();
        }
        catch (Exception e)
        {
            globalTracer.traceWarn(moduleName, "FieldZeroHeading file not found.");
            return null;
        }
    }   //getFieldZeroHeading

    /**
     * This method saves the compass heading value when the robot is facing field zero.
     */
    public void saveFieldZeroCompassHeading()
    {
        if (robotDrive != null && robotDrive.imu != null && robotInfo.imuType == ImuType.NavX)
        {
            try (PrintStream out = new PrintStream(new FileOutputStream(RobotParams.Robot.FIELD_ZERO_CAL_FILE)))
            {
                double fieldZeroHeading = ((FrcAHRSGyro) robotDrive.imu).ahrs.getCompassHeading();

                out.println(fieldZeroHeading);
                out.close();
                globalTracer.traceInfo(moduleName, "FieldZeroCompassHeading=" + fieldZeroHeading);
            }
            catch (FileNotFoundException e)
            {
                e.printStackTrace();
            }
        }
    }   //saveFieldZeroCompassHeading

    /**
     * This method sets the robot's absolute field position. This is typically called at the beginning of a match for
     * robot localization. The provided pose should be the robot's starting position. If null, it will try to get the
     * robot start pose from the auto choices on the dashboard. Optionally, the caller can set useCompassHeading to
     * true for using compass heading to determine the true robot heading. This only works if the robot has been
     * calibrated on the competition field for its field zero position.
     * Note: if reading the field zero calibration file failed, it will behave as if useCompassHeading is false.
     *
     * @param pose speicifies the robot's starting position on the field.
     * @param useCompassHeading specifies true to use compass to determine the robot's true heading, false otherwise.
     */
    public void setFieldPosition(TrcPose2D pose, boolean useCompassHeading)
    {
        TrcPose2D robotPose;

        if (pose == null)
        {
            int startPos = FrcAuto.autoChoices.getStartPos().value;
            robotPose = (FrcAuto.autoChoices.getAlliance() == Alliance.Blue?
                RobotParams.Game.startPoses[0][startPos]: RobotParams.Game.startPoses[1][startPos]).clone();
        }
        else
        {
            robotPose = pose.clone();
        }

        if (useCompassHeading && robotDrive.imu != null && robotInfo.imuType == ImuType.NavX)
        {
            Double fieldZero = getFieldZeroCompassHeading();

            if (fieldZero != null)
            {
                robotPose.angle = ((FrcAHRSGyro) robotDrive.imu).ahrs.getCompassHeading() - fieldZero;
            }
        }

        robotDrive.driveBase.setFieldPosition(robotPose);
    }   //setFieldPosition

    /**
     * This method sets the robot's absolute field position. This is typically called at the beginning of a match for
     * robot localization. The provided pose should be the robot's starting position. If null, it will try to get the
     * robot start pose from the auto choices on the dashboard. Optionally, the caller can set  useCompassHeading to
     * true for using compass heading to determine the true robot heading. This only works if the robot has been
     * calibrated on the competition field for its field zero position.
     * Note: if reading the field zero calibration file failed, it will behave as if useCompassHeading is false.
     *
     * @param useCompassHeading specifies true to use compass to determine the robot's true heading, false otherwise.
     */
    public void setFieldPosition(boolean useCompassHeading)
    {
        setFieldPosition(null, useCompassHeading);
    }   //setFieldPosition

    /**
     * This method sets the drive orientation mode and update the LEDs if necessary.
     *
     * @param orientation specifies the drive orientation.
     * @param resetHeading specifies true to also reset the robot heading, only valid for FIELD mode.
     */
    public void setDriveOrientation(DriveOrientation orientation, boolean resetHeading)
    {
        if (robotDrive != null)
        {
            robotDrive.driveBase.setDriveOrientation(orientation, resetHeading);
            if (ledIndicator != null)
            {
                ledIndicator.setDriveOrientation(orientation);
            }
        }
    }   //setDriveOrientation

    /**
     * This method uses the detect AprilTag to relocalize the robot's position.
     *
     * @param aprilTagObj specifies the detected AprilTag object to be used for relocalization.
     * @return true if relocalization is successful, false otherwise.
     */
    public boolean relocalizeRobotByAprilTag(FrcPhotonVision.DetectedObject aprilTagObj)
    {
        boolean success = false;
        // Use AprilTag's location to re-localize the robot.
        if (aprilTagObj.robotPose != null)
        {
            robotDrive.driveBase.setFieldPosition(aprilTagObj.robotPose, false);
            globalTracer.traceInfo(moduleName, "Using AprilTag to re-localize to " + aprilTagObj.robotPose);
            success = true;
        }

        return success;
    }   //relocalizeRobotByAprilTag

    /**
     * This method re-localizes the robot with AprilTag vision reported info.
     *
     * @param aprilTagObj specifies the detected AprilTag object.
     */
    public void relocalize(FrcPhotonVision.DetectedObject aprilTagObj)
    {
        // Use vision to relocalize robot's position.
        int aprilTagId = aprilTagObj.target.getFiducialId();
        TrcPose2D robotEstimatedPose = aprilTagObj.robotPose;

        if (robotEstimatedPose == null)
        {
            // PhotonVision pose estimator failed to return estimatedPose?! Calculate the pose ourselves.
            robotEstimatedPose = photonVisionFront.getRobotFieldPose(aprilTagObj, false);
            globalTracer.traceInfo(
                moduleName, "Relocalize Robot: aprilTagId=" + aprilTagId +
                ", robotEstimatedPoseFromAprilTag=" + robotEstimatedPose);
        }

        TrcPose2D robotPose = robotDrive.driveBase.getFieldPosition();
        double xDelta = robotPose.x - robotEstimatedPose.x;
        double yDelta = robotPose.y - robotEstimatedPose.y;
        double error = TrcUtil.magnitude(xDelta, yDelta);
        if (error > RobotParams.Vision.GUIDANCE_ERROR_THRESHOLD && error < 96.00)
        {
            robotDrive.driveBase.setFieldPosition(robotEstimatedPose, false);
            globalTracer.traceInfo(
                moduleName, "Relocalize Robot: AprilTagId=" + aprilTagId + ", error=" + error +
                ", robotPose=" + robotPose + ", relocalizePose=" + robotEstimatedPose);
        }
        else
        {
            globalTracer.traceInfo(
                moduleName, "Relocalize Robot: aprilTagId=" + aprilTagId + ", error=" + error +
                " (error too large or small to relocalize).");
        }
    }   //relocalize

    /**
     * This method is called when Comm Status changes state. This is an indication of losing or regaining comm.
     *
     * @param context specifies true for comm connected, false for comm disconnected.
     */
    private void commStatusCallback(Object context)
    {
        Boolean commStatus = (Boolean) context;

        if (!commStatus)
        {
            // We lost comm, do emergency shutdown to prevent damage.
            if (robotDrive != null && robotDrive instanceof FrcSwerveDrive)
            {
                cancelAll();
                ((FrcSwerveDrive) robotDrive).setXModeEnabled(null, true);
                globalTracer.traceInfo(moduleName, "***** Putting robot in X-Mode. *****");
            }
        }
    }   //commStatusCallback

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param x specifies x position in the blue alliance in the specified unit.
     * @param y specifies y position in the blue alliance in the specified unit.
     * @param heading specifies heading in the blue alliance in degrees.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(double x, double y, double heading, Alliance alliance)
    {
        TrcPose2D newPose = new TrcPose2D(x, y, heading);

        if (alliance == Alliance.Red)
        {
            // Translate blue alliance pose to red alliance pose.
            if (RobotParams.Game.fieldIsMirrored)
            {
                // Mirrored field.
                double angleDelta = (newPose.angle - 90.0)*2.0;
                newPose.angle -= angleDelta;
                newPose.y = RobotParams.Field.LENGTH - newPose.y;
            }
            else
            {
                // Symmetrical field.
                newPose.x = -RobotParams.Field.WIDTH - newPose.x;
                newPose.y = RobotParams.Field.LENGTH - newPose.y;
                newPose.angle = (newPose.angle + 180.0) % 360.0;
            }
        }

        return newPose;
    }   //adjustPoseByAlliance

    /**
     * This method adjusts the given pose in the blue alliance to be the specified alliance.
     *
     * @param pose specifies pose in the blue alliance in the specified unit.
     * @param alliance specifies the alliance to be converted to.
     * @return pose adjusted to be in the specified alliance in inches.
     */
    public TrcPose2D adjustPoseByAlliance(TrcPose2D pose, Alliance alliance)
    {
        return adjustPoseByAlliance(pose.x, pose.y, pose.angle, alliance);
    }   //adjustPoseByAlliance

    //
    // Getters for sensor data.
    //

    /**
     * This method returns the pressure value from the pressure sensor.
     *
     * @return pressure value.
     */
    public double getPressure()
    {
        return pressureSensor != null? (pressureSensor.getVoltage() - 0.5) * 50.0: 0.0;
    }   //getPressure

}   //class Robot
