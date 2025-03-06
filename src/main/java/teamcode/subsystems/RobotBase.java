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

package teamcode.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frclib.drivebase.FrcRobotDrive;
import frclib.drivebase.FrcSwerveDrive;
import frclib.drivebase.FrcSwerveDrive.SteerEncoderType;
import frclib.motor.FrcCANTalonFX;
import frclib.motor.FrcMotorActuator.MotorType;
import teamcode.RobotParams;
import teamcode.RobotParams.HwConfig;
import trclib.controller.TrcPidController.PidCoefficients;
import trclib.drivebase.TrcDriveBase.OdometryType;
import trclib.pathdrive.TrcPose3D;
import trclib.robotcore.TrcDbgTrace;
import trclib.sensor.TrcEncoder;

/**
 * This class creates the appropriate Robot Drive Base according to the specified robot type.
 */
public class RobotBase
{
    private static final String moduleName = RobotBase.class.getSimpleName();

    /**
     * When the season starts, the competition robot may not be ready for programmers. It's crucial to save time by
     * developing code on robots of previous seasons. By adding previous robots to the list of RobotType, one can
     * easily switch the code to handle different robots.
     */
    public enum RobotType
    {
        // This is useful for developing Vision code where all you need is a Robot Controller and camera.
        VisionOnly,
        // Crescendo robot.
        MaestroRobot,
        // Reefscape robot.
        ReefscapeRobot
    }   //enum RobotType

    /**
     * This class contains the parameters of the front camera.
     */
    public static class MaestroFrontCamParams extends FrcRobotDrive.VisionInfo
    {
        public MaestroFrontCamParams()
        {
            camName = "OV9281";
            camImageWidth = 1280;
            camImageHeight = 800;
            if (RobotParams.Preferences.invertedRobot)
            {
                camXOffset = 3.5;               // Inches to the right from robot center
                camYOffset = 2.375;             // Inches forward from robot center
                camYaw = 180.0;                 // degrees clockwise from robot front
            }
            else
            {
                camXOffset = -3.5;              // Inches to the right from robot center
                camYOffset = -2.375;            // Inches forward from robot center
                camYaw = 0.0;                   // degrees clockwise from robot front
            }
            camZOffset = 23.125;                // Inches up from the floor
            camPitch = 33.0;                    // degrees up from horizontal
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //MaestroFrontCamParams
    }   //class MaestroFrontCamParams

    /**
     * This class contains the parameters of the back camera.
     */
    public static class MaestroBackCamParams extends FrcRobotDrive.VisionInfo
    {
        public MaestroBackCamParams()
        {
            camName = "OV9782";
            camImageWidth = 1280;
            camImageHeight = 800;
            if (RobotParams.Preferences.invertedRobot)
            {
                camXOffset = 0.5;               // Inches to the right from robot center
                camYOffset = 5.375;             // Inches forward from robot center
                camYaw = 0.0;                   // degrees clockwise from robot front
            }
            else
            {
                camXOffset = -0.5;              // Inches to the right from robot center
                camYOffset = -5.375;            // Inches forward from robot center
                camYaw = 180.0;                 // degrees clockwise from robot front
            }
            camZOffset = 20.0;                  // Inches up from the floor
            camPitch = -17.5;                   // degrees up from horizontal
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //MaestroBackCamParams
    }   //class MaestroBackCamParams

    public static class MaestroParams extends FrcSwerveDrive.SwerveInfo
    {
        public final double FALCON_MAX_RPM                      = 6380.0;
        public final double DRIVE_GEAR_RATIO                    = 6.75;
        public final double DRIVE_WHEEL_DIAMETER                = 3.9326556997620689090425924610785;    // inches
        public final double STEER_GEAR_RATIO                    = 15.43;

        public MaestroParams()
        {
            robotName = "Maestro";
            // Robot Dimensions.
            robotLength = RobotParams.Robot.ROBOT_LENGTH;
            robotWidth = RobotParams.Robot.ROBOT_WIDTH;
            wheelBaseLength = 23.25;
            wheelBaseWidth = 23.25;
            // Gyro parameters.
            imuName = "NavX";
            imuType = FrcRobotDrive.ImuType.NavX;
            imuPort = NavXComType.kMXP_SPI;
            // Drive Motor parameters.
            driveMotorType = MotorType.CanTalonFx;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorIds = new int[] {
                HwConfig.CANID_LFDRIVE_MOTOR, HwConfig.CANID_RFDRIVE_MOTOR,
                HwConfig.CANID_LBDRIVE_MOTOR, HwConfig.CANID_RBDRIVE_MOTOR};
            if (RobotParams.Preferences.invertedRobot)
            {
                // To test Reefscape code on Maestro, we are inverting the robot direction. (i.e. robot front is now
                // robot back).
                driveMotorInverted = new boolean[] {true, true, true, true};
            }
            else
            {
                driveMotorInverted = new boolean[] {false, false, false, false};
            }
            odometryType = OdometryType.MotorOdometry;
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            xDrivePosScale = yDrivePosScale = 0.91311455; //DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_GEAR_RATIO;    // inch/rev
            // Robot Drive Characteristics
            robotMaxVelocity = 171.0;           // inch/sec
            robotMaxAcceleration = 23000.0;     // inch/sec sq
            robotMaxDeceleration = robotMaxAcceleration;
            robotMaxTurnRate = 1450.0;          // degree/sec
            profiledMaxVelocity = 157.48;       // inch/sec
            profiledMaxAcceleration = 10000.0;  // inch/sec sq
            profiledMaxDeceleration = profiledMaxAcceleration;
            profiledMaxTurnRate = 180.0;        // degree/sec
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 2.0;
            xDrivePidCoeffs = yDrivePidCoeffs = new PidCoefficients(0.017, 0.0, 0.0025, 0.0, 5.0);
            xDrivePidPowerLimit = yDrivePidPowerLimit = 0.5;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = 0.5;  // %power per sec
            turnPidCoeffs = new PidCoefficients(0.0065, 0.0, 0.0004, 0.0, 10.0);
            turnPidPowerLimit = 1.0;
            turnMaxPidRampRate = 1.0;           // %power per sec
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PidDrive Parameters
            usePidDrive = true;
            enablePidDriveSquareRootPid = false;
            // PurePursuit Parameters
            usePurePursuitDrive = true;
            enablePurePursuitDriveSquareRootPid = false;
            ppdFollowingDistance = 10.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1.0 / robotMaxVelocity, 0.0);
            fastModeEnabled = true;
            // Vision
            if (RobotParams.Preferences.invertedRobot)
            {
                // To Test Reefscape code on Maestro, we are making the Maestro robot back as robot front.
                cam1 = new MaestroBackCamParams();
                cam2 = new MaestroFrontCamParams();
            }
            else
            {
                cam1 = new MaestroFrontCamParams();
                cam2 = new MaestroBackCamParams();
            }
            // Miscellaneous
            ledName = "LED";
            ledChannel = HwConfig.PWM_CHANNEL_LED;
            numLEDs = HwConfig.NUM_LEDS;
            // Steer Encoder parameters.
            steerEncoderType = SteerEncoderType.Canandmag;
            steerEncoderNames = new String[] {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
            steerEncoderIds = new int[] {
                HwConfig.CANID_LFSTEER_ENCODER, HwConfig.CANID_RFSTEER_ENCODER,
                HwConfig.CANID_LBSTEER_ENCODER, HwConfig.CANID_RBSTEER_ENCODER};
            steerEncoderInverted = new boolean[] {false, false, false, false};
            steerEncoderZeros = new double[] {0.0, 0.0, 0.0, 0.0};
            steerZerosFilePath = RobotParams.Robot.STEER_ZERO_CAL_FILE;
            // Steer Motor parameters.
            steerMotorType = MotorType.CanTalonFx;
            steerMotorNames = new String[] {"lfSteerMotor", "rfSteerMotor", "lbSteerMotor", "rbSteerMotor"};
            steerMotorIds = new int[] {
                HwConfig.CANID_LFSTEER_MOTOR, HwConfig.CANID_RFSTEER_MOTOR,
                HwConfig.CANID_LBSTEER_MOTOR, HwConfig.CANID_RBSTEER_MOTOR};
            steerMotorInverted = new boolean[] {false, false, false, false};
            steerMotorPidCoeffs = new PidCoefficients(3.0, 0.0, 0.0, 0.0, 0.0);
            steerMotorPidTolerance = 0.5; // in degrees
            // Swerve Module parameters.
            swerveModuleNames = new String[] {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};
            driveGearRatio = DRIVE_GEAR_RATIO;
            steerGearRatio = STEER_GEAR_RATIO;
            steerPositionScale = 360.0 / steerGearRatio;
            //
            // WPILib Parameters.
            //
            invertGyro = true;   // Always ensure Gyro is CCW+ CW-
            // Drivetrain Constants
            wheelBase = Units.inchesToMeters(wheelBaseLength);
            trackWidth = Units.inchesToMeters(wheelBaseWidth);
            wheelCircumference = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER * Math.PI);
            // Swerve Kinematics
            // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
            swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
            // Meters per Second
            maxSpeed = Units.inchesToMeters(profiledMaxVelocity);
            // Radians per Second
            maxAngularVelocity = Units.degreesToRadians(profiledMaxTurnRate);
            driveKs = 0.32;
            driveKv = 1.51;
            driveKa = 0.27;

            // double steerScaleDegPerRot = 360.0 / STEER_GEAR_RATIO;
            // // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
            // double steerMaxVelocity = (FALCON_MAX_RPM * 0.81 / STEER_GEAR_RATIO /60.0) * 360.0;
            // // kF set to Motion Magic recommendation.
        }   //MaestroParams
    }   //class MaestroParams

    public static class ReefscapeFrontCamParams extends FrcRobotDrive.VisionInfo
    {
        public ReefscapeFrontCamParams()
        {
            camName = "FrontOV9782";
            camImageWidth = 1280;
            camImageHeight = 800;
            camXOffset = -0.1875;               // Inches to the right from robot center
            camYOffset = 5.9375;                // Inches forward from robot center
            camZOffset = 6.75;                  // Inches up from the floor
            camYaw = 0.0;                       // degrees clockwise from robot front
            camPitch = 16.0;                    // degrees up from horizontal
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //ReefscapeFrontCamParams
    }   //class ReefscapeFrontCamParams

    /**
     * This class contains the parameters of the back camera.
     */
    public static class ReefscapeBackCamParams extends FrcRobotDrive.VisionInfo
    {
        public ReefscapeBackCamParams()
        {
            camName = "BackOV9782";
            camImageWidth = 1280;
            camImageHeight = 800;
            camXOffset = 0.0;                  // Inches to the right from robot center
            camYOffset = 0.0;                // Inches forward from robot center
            camZOffset = 20.0;                  // Inches up from the floor
            camYaw = 180.0;                     // degrees clockwise from robot front
            camPitch = 0.0;                   // degrees up from horizontal
            camRoll = 0.0;
            robotToCam = new Transform3d(
                new Translation3d(Units.inchesToMeters(camYOffset),
                                  -Units.inchesToMeters(camXOffset),
                                  Units.inchesToMeters(camZOffset)),
                new Rotation3d(Units.degreesToRadians(camRoll),
                               Units.degreesToRadians(-camPitch),
                               Units.degreesToRadians(-camYaw)));
            camPose = new TrcPose3D(camXOffset, camYOffset, camZOffset, camYaw, camPitch, camRoll);
        }   //ReefscapeBackCamParams
    }   //class ReefscapeBackCamParams

    /**
     * This class contains the Swerve Robot Parameters.
     */
    public static class ReefscapeRobotParams extends FrcSwerveDrive.SwerveInfo
    {
        public final double FALCON_MAX_RPM                      = 6380.0;
        public final double DRIVE_GEAR_RATIO                    = 5.60;//6.75;
        public final double DRIVE_WHEEL_DIAMETER                = 3.9326556997620689090425924610785;    // inches
        public final double STEER_GEAR_RATIO                    = 13.3714;

        public ReefscapeRobotParams()
        {
            robotName = "ReefscapeRobot";
            // Robot Dimensions.
            robotLength = RobotParams.Robot.ROBOT_LENGTH;
            robotWidth = RobotParams.Robot.ROBOT_WIDTH;
            wheelBaseLength = 23.25;
            wheelBaseWidth = 23.25;
            // Gyro parameters.
            imuName = "NavX";
            imuType = FrcRobotDrive.ImuType.NavX;
            imuPort = NavXComType.kMXP_SPI;
            // Drive Motor parameters.
            driveMotorType = MotorType.CanTalonFx;
            driveMotorNames = new String[] {"lfDriveMotor", "rfDriveMotor", "lbDriveMotor", "rbDriveMotor"};
            driveMotorIds = new int[] {
                HwConfig.CANID_LFDRIVE_MOTOR, HwConfig.CANID_RFDRIVE_MOTOR,
                HwConfig.CANID_LBDRIVE_MOTOR, HwConfig.CANID_RBDRIVE_MOTOR};
            driveMotorInverted = new boolean[] {false, false, false, false};
            odometryType = OdometryType.MotorOdometry;
            // Absolute Odometry
            absoluteOdometry = null;
            // Drive Motor Odometry
            xDrivePosScale = yDrivePosScale = DRIVE_WHEEL_DIAMETER * Math.PI / DRIVE_GEAR_RATIO;    // inch/rev
            // Robot Drive Characteristics
            robotMaxVelocity = 201.0;           // inch/sec
            robotMaxAcceleration = 6935.0;     // inch/sec sq
            robotMaxDeceleration = robotMaxAcceleration;
            robotMaxTurnRate = 478.0;          // degree/sec


            profiledMaxVelocity = 110.0;       // inch/sec
            profiledMaxAcceleration = 200.0;  // inch/sec sq
            profiledMaxDeceleration = 90.0;
            profiledMaxTurnRate = 200.0;     // degree/sec
            // DriveBase PID Parameters
            drivePidTolerance = 1.0;
            turnPidTolerance = 1.0;
            xDrivePidCoeffs = yDrivePidCoeffs = new PidCoefficients(0.012, 0.0, 0.0012, 0.0, 5.0);
            xDrivePidPowerLimit = yDrivePidPowerLimit = 0.5;
            xDriveMaxPidRampRate = yDriveMaxPidRampRate = 0.5;  // %power per sec
            turnPidCoeffs = new PidCoefficients(0.0044, 0.0, 0.0, 0.0, 0.0);
            turnPidPowerLimit = 1.0;
            turnMaxPidRampRate = 1.0;           // %power per sec
            // PID Stall Detection
            pidStallDetectionEnabled = true;
            // PidDrive Parameters
            usePidDrive = true;
            enablePidDriveSquareRootPid = false;
            // PurePursuit Parameters
            usePurePursuitDrive = true;
            enablePurePursuitDriveSquareRootPid = false;
            ppdFollowingDistance = 10.0;
            velPidCoeffs = new PidCoefficients(0.0, 0.0, 0.0, 1/profiledMaxVelocity, 0.0);
            fastModeEnabled = true; 
            // Vision
            cam1 = new ReefscapeFrontCamParams();
            cam2 = new ReefscapeBackCamParams();
            // Miscellaneous
            ledName = "LED";
            ledChannel = HwConfig.PWM_CHANNEL_LED;
            numLEDs = HwConfig.NUM_LEDS;
            // Steer Encoder parameters.
            steerEncoderType = SteerEncoderType.Canandmag;
            steerEncoderNames = new String[] {"lfSteerEncoder", "rfSteerEncoder", "lbSteerEncoder", "rbSteerEncoder"};
            steerEncoderIds = new int[] {
                HwConfig.CANID_LFSTEER_ENCODER, HwConfig.CANID_RFSTEER_ENCODER,
                HwConfig.CANID_LBSTEER_ENCODER, HwConfig.CANID_RBSTEER_ENCODER};
            steerEncoderInverted = new boolean[] {false, false, false, false};
            steerEncoderZeros = new double[] {0.0, 0.0, 0.0, 0.0};
            steerZerosFilePath = RobotParams.Robot.STEER_ZERO_CAL_FILE;
            // Steer Motor parameters.
            steerMotorType = MotorType.CanTalonFx;
            steerMotorNames = new String[] {"lfSteerMotor", "rfSteerMotor", "lbSteerMotor", "rbSteerMotor"};
            steerMotorIds = new int[] {
                HwConfig.CANID_LFSTEER_MOTOR, HwConfig.CANID_RFSTEER_MOTOR,
                HwConfig.CANID_LBSTEER_MOTOR, HwConfig.CANID_RBSTEER_MOTOR};
            steerMotorInverted = new boolean[] {false, false, false, false};
            steerMotorPidCoeffs = new PidCoefficients(3.0, 0.0, 0.0, 0.0, 0.0);
            steerMotorPidTolerance = 0.5; // in degrees
            // Swerve Module parameters.
            swerveModuleNames = new String[] {"lfWheel", "rfWheel", "lbWheel", "rbWheel"};
            driveGearRatio = DRIVE_GEAR_RATIO;
            steerGearRatio = STEER_GEAR_RATIO;
            steerPositionScale = 360.0 / steerGearRatio;
            //
            // WPILib Parameters.
            //
            invertGyro = true;   // Always ensure Gyro is CCW+ CW-
            // Drivetrain Constants
            wheelBase = Units.inchesToMeters(wheelBaseLength);
            trackWidth = Units.inchesToMeters(wheelBaseWidth);
            wheelCircumference = Units.inchesToMeters(DRIVE_WHEEL_DIAMETER * Math.PI);
            // Swerve Kinematics
            // No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve
            swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
            // Meters per Second
            maxSpeed = Units.inchesToMeters(profiledMaxVelocity);
            // Radians per Second
            maxAngularVelocity = Units.degreesToRadians(profiledMaxTurnRate);
            driveKs = 0.32;
            driveKv = 1.51;
            driveKa = 0.27;

            // double steerScaleDegPerRot = 360.0 / STEER_GEAR_RATIO;
            // // ((theoretical max rpm * speed loss constant / gear ratio) / 60 sec/min) * 360 deg/rev
            // double steerMaxVelocity = (FALCON_MAX_RPM * 0.81 / STEER_GEAR_RATIO /60.0) * 360.0;
            // // kF set to Motion Magic recommendation.
        }   //ReefscapeRobotParams
    }   //class ReefscapeRobotParams

    public static class VisionOnlyParams extends FrcRobotDrive.RobotInfo
    {
        public VisionOnlyParams()
        {
            robotName = "VisionOnly";
            // Vision
            cam1 = new ReefscapeFrontCamParams();
            cam2 = new ReefscapeBackCamParams();
        }   //VisionOnlyParams
    }   //class VisionOnlyParams

    public static final class AutoConstants
    {
        //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 0.0;//0.00149
        public static final double kPYController = 0.0;
        public static final double kPThetaController = 0.0;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }   //class AutoConstants

    private final FrcRobotDrive.RobotInfo robotInfo;
    private final FrcRobotDrive robotDrive;

    /**
     * Constructor: Create an instance of the object.
     */
    public RobotBase()
    {
        switch (RobotParams.Preferences.robotType)
        {
            case VisionOnly:
                robotInfo = new VisionOnlyParams();
                robotDrive = null;
                break;

            case MaestroRobot:
                robotInfo = new MaestroParams();
                robotDrive = RobotParams.Preferences.useDriveBase?
                    new FrcSwerveDrive((MaestroParams) robotInfo): null;
                break;

            case ReefscapeRobot:
                robotInfo = new ReefscapeRobotParams();
                robotDrive = RobotParams.Preferences.useDriveBase?
                    new FrcSwerveDrive((ReefscapeRobotParams) robotInfo): null;
                break;

            default:
                robotInfo = null;
                robotDrive = null;
                break;
        }
        configureRobotDrive();
    }   //RobotBase

    /**
     * This method returns the created RobotInfo object.
     *
     * @return created robot info.
     */
    public FrcRobotDrive.RobotInfo getRobotInfo()
    {
        return robotInfo;
    }   //getRobotInfo

    /**
     * This method returns the created RobotBase object.
     *
     * @return created robot drive.
     */
    public FrcRobotDrive getRobotDrive()
    {
        return robotDrive;
    }   //getRobotDrive

    /**
     * This method configures robotDrive with implementation details.
     */
    private void configureRobotDrive()
    {
        if (robotDrive != null)
        {
            if (robotDrive instanceof FrcSwerveDrive)
            {
                FrcSwerveDrive swerveDrive = (FrcSwerveDrive) robotDrive;
                FrcSwerveDrive.SwerveInfo swerveInfo = (FrcSwerveDrive.SwerveInfo) robotInfo;
                // Prevent Krakens from browning out.
                for (int i = 0; i < swerveInfo.driveMotorNames.length; i++)
                {
                    swerveDrive.driveMotors[i].setCloseLoopRampRate(0.02);
                    swerveDrive.driveMotors[i].setCurrentLimit(40.0, 45.0, 0.2);
                    swerveDrive.driveMotors[i].setStatorCurrentLimit(55.0);
                }
                // Sync absolute encoders to steer motor internal encoders.
                for (int i = 0; i < swerveInfo.steerEncoderNames.length; i++)
                {
                    syncSteerEncoder((FrcSwerveDrive.SwerveInfo) robotInfo, i);
                }
            }
        }
    }   //configureRobotDrive

    /**
     * This method reads the absolute steering encoder and synchronize the steering motor encoder with it.
     *
     * @param swerveInfo specifies the swerve drive parameters.
     * @param index specifies the swerve module index.
     */
    private void syncSteerEncoder(FrcSwerveDrive.SwerveInfo swerveInfo, int index)
    {
        // Note this method is implementation specific. If your implementation is not with an absolute encoder that
        // syncs with a TalonFX motor, you need to modify this method accordingly.
        FrcSwerveDrive swerveDrive = (FrcSwerveDrive) robotDrive;
        TrcEncoder steerEncoder = swerveDrive.steerEncoders[index];
        FrcCANTalonFX steerMotor = (FrcCANTalonFX)swerveDrive.steerMotors[index];
        // getPosition returns a value in the range of 0 to 1.0 of one revolution.
        double motorEncoderPos = steerEncoder.getScaledPosition() * swerveInfo.steerGearRatio;
        StatusCode statusCode = steerMotor.motor.setPosition(motorEncoderPos);

        if (statusCode != StatusCode.OK)
        {
            TrcDbgTrace.globalTraceWarn(
                moduleName,
                swerveInfo.swerveModuleNames[index] + ": TalonFx.setPosition failed (code=" + statusCode +
                ", pos=" + motorEncoderPos + ").");
        }

        double actualEncoderPos = steerMotor.motor.getPosition().getValueAsDouble();
        if (Math.abs(motorEncoderPos - actualEncoderPos) > 0.1)
        {
            TrcDbgTrace.globalTraceWarn(
                moduleName,
                swerveInfo.swerveModuleNames[index] +
                ": Steer encoder out-of-sync (expected=" + motorEncoderPos + ", actual=" + actualEncoderPos + ")");
        }
    }   //syncSteerEncoder

}   //class RobotDrive
