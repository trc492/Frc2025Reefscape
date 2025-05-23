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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import teamcode.subsystems.RobotBase.RobotType;
import trclib.pathdrive.TrcPose2D;

/**
 * This class contains robot and subsystem constants and parameters.
 */
public class RobotParams
{
    /**
     * This class contains robot preferences. It enables/disables various robot features. This is especially useful
     * during robot development where some subsystems may not be available or ready yet. By disabling unavailable
     * subsystems, one can test the rest of the robot without the fear of code crashing when some subsystems are not
     * found.
     */
    public static class Preferences
    {
        // Global config
        public static final RobotType robotType                 = RobotType.ReefscapeRobot;
        public static final boolean inCompetition               = false;    //true
        public static final boolean hybridMode                  = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useCommStatusMonitor        = false;
        public static final boolean invertedRobot               = false;
        // Dashboard Update
        public static final boolean updateDashboard             = !inCompetition;
        public static final boolean showDriveBase               = false;
        public static final boolean debugDriveBase              = false;
        public static final boolean showPidDrive                = false;
        public static final boolean showDrivePower              = true;
        public static final boolean showVision                  = true;
        public static final boolean showSubsystems              = true;
        // Sensors and Indicators
        public static final boolean useNavX                     = true;
        public static final boolean usePdp                      = false;
        public static final boolean usePressureSensor           = false;
        public static final boolean useLED                      = true;
        public static final boolean useRumble                   = true;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean usePhotonVision             = true;
        public static final boolean useOpenCvVision             = false;
        public static final boolean useStreamCamera             = false;
        public static final boolean doVisionRelocalize          = false;
        // Drive Base
        public static final boolean useDriveBase                = true;
        public static final boolean useVelocityControl          = false;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
        public static final boolean useCoralArm                 = true;
        public static final boolean useCoralGrabber             = useCoralArm;
        public static final boolean useClimber                  = true;
        public static final boolean useClimberGrabber           = true;
        public static final boolean useElevator                 = true;
        public static final boolean useElevatorArm              = true;
    }   //class Preferences

    /**
     * This class contains the Robot Hardware Configurations.
     */
    public static class HwConfig
    {
        // Joystick ports.
        public static final int XBOX_DRIVER_CONTROLLER          = 0;
        public static final int XBOX_OPERATOR_CONTROLLER        = 1;
        // CAN IDs.
        // Drive Motor CAN IDs.
        public static final int CANID_LFDRIVE_MOTOR             = 3;    // Orange
        public static final int CANID_RFDRIVE_MOTOR             = 4;    // Yellow
        public static final int CANID_LBDRIVE_MOTOR             = 5;    // Green
        public static final int CANID_RBDRIVE_MOTOR             = 6;    // Blue
        // Swerve CAN IDs.
        public static final int CANID_LFSTEER_MOTOR             = 13;   // Orange
        public static final int CANID_RFSTEER_MOTOR             = 14;   // Yellow
        public static final int CANID_LBSTEER_MOTOR             = 15;   // Green
        public static final int CANID_RBSTEER_MOTOR             = 16;   // Blue
        public static final int CANID_LFSTEER_ENCODER           = 23;   // Orange
        public static final int CANID_RFSTEER_ENCODER           = 24;   // Yellow
        public static final int CANID_LBSTEER_ENCODER           = 25;   // Green
        public static final int CANID_RBSTEER_ENCODER           = 26;   // Blue
        // Subsystem Motor CAN IDs.
        public static final int CANID_CORALARM_MOTOR            = 7;    // Purple
        public static final int CANID_CLIMBERARM_MOTOR          = 8;    // Gray
        public static final int CANID_ELEVATOR_MOTOR            = 9;    // White
        public static final int CANID_CORALGRABBER_MOTOR        = 17;   // Purple
        public static final int CANID_CLIMBERGRABBER_MOTOR      = 18;   // Gray
        public static final int CANID_CLIMBERARM_ENCODER        = 28;   // Gray
        // Miscellaneous CAN IDs.
        public static final int CANID_PDP                       = 30;
        public static final int CANID_PCM                       = 31;
        // Analog Input ports (not used).
        public static final int AIN_ULTRASONIC                  = 0;
        public static final int AIN_PRESSURE_SENSOR             = 0;
        public static final int AIN_LFSTEER_ENCODER             = 0;
        public static final int AIN_RFSTEER_ENCODER             = 1;
        public static final int AIN_LBSTEER_ENCODER             = 2;
        public static final int AIN_RBSTEER_ENCODER             = 3;
        // Digital Input/Output ports.
        public static final int DIO_CORAL_GRABBER_SENSOR        = 0;
        public static final int DIO_CLIMBER_GRABBER_SENSOR      = 3;
        // PWM channels.
        public static final int NUM_LEDS                        = 30;
        public static final int PWM_CHANNEL_LED                 = 0;
        // Relay channels.

        // Pneumatic channels.

        // PDP Channels.
        // Drive Base PDP Channels.
        public static final ModuleType PDP_MODULE_TYPE          = ModuleType.kRev;
        public static final int PDP_CHANNEL_LFDRIVE_MOTOR       = 11;
        public static final int PDP_CHANNEL_RFDRIVE_MOTOR       = 5;
        public static final int PDP_CHANNEL_LBDRIVE_MOTOR       = 13;
        public static final int PDP_CHANNEL_RBDRIVE_MOTOR       = 3;
        public static final int PDP_CHANNEL_LFSTEER_MOTOR       = 10;
        public static final int PDP_CHANNEL_RFSTEER_MOTOR       = 6;
        public static final int PDP_CHANNEL_LBSTEER_MOTOR       = 12;
        public static final int PDP_CHANNEL_RBSTEER_MOTOR       = 4;
        // Miscellaneous PDP Channels.
        public static final int PDP_CHANNEL_ROBORIO             = 20;
        public static final int PDP_CHANNEL_VRM                 = 18;
        public static final int PDP_CHANNEL_PCM                 = 19;
        public static final int PDP_CHANNEL_RADIO_POE           = 22;
        public static final int PDP_CHANNEL_ETHERNET_SWITCH     = 21;
        public static final int PDP_CHANNEL_CAMERA              = 0;
        public static final int PDP_CHANNEL_LED                 = 14;

        public static final double BATTERY_CAPACITY_WATT_HOUR   = 18.0*12.0;

        // Ultrasonic sensors.
        // public static final double SONAR_INCHES_PER_VOLT        = 1.0/0.0098; //9.8mV per inch
        // public static final double SONAR_ERROR_THRESHOLD        = 50.0; //value should not jump 50-in per time slice.
    }   //class HwConfig

    /**
     * This class contains Robot parameters.
     */
    public static class Robot
    {
        public static final String TEAM_FOLDER_PATH             = "/home/lvuser/trc492";
        public static final String LOG_FOLDER_PATH              = TEAM_FOLDER_PATH + "/tracelogs";
        public static final String STEER_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/SteerZeroCalibration.txt";
        public static final String FIELD_ZERO_CAL_FILE          = TEAM_FOLDER_PATH + "/FieldZeroCalibration.txt";
        public static final double DASHBOARD_UPDATE_INTERVAL    = 0.1;      // in msec
        public static final String ROBOT_CODEBASE               = "Reefscape";
        public static final double ROBOT_LENGTH                 = 35.5;
        public static final double ROBOT_WIDTH                  = 35.5;
        public static final double DRIVE_RAMP_RATE              = 0.25;
    }   //class Robot

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        //
        // Game time.
        //
        public static final double AUTONOMOUS_PERIOD            = 15.0;     // in seconds
        public static final double TELEOP_PERIOD                = 135.0;    // in seconds
        public static final double CLIMB_PERIOD                 = 20.0;     // in seconds
        //
        // Robot starting positions.
        //
        public static final double STARTPOS_BLUE_Y              = 297.5 - Robot.ROBOT_LENGTH / 2.0; // 279.75
        public static final double STARTPOS_BLUE_PROCESSOR_X    = -241.73;
        public static final double STARTPOS_BLUE_CENTER_X       = -Field.WIDTH / 2.0; // -158.5
        public static final double STARTPOS_BLUE_FAR_X          = -75.2;
        public static final TrcPose2D STARTPOS_BLUE_PROCESSOR   =
            new TrcPose2D(STARTPOS_BLUE_PROCESSOR_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D STARTPOS_BLUE_CENTER      =
            new TrcPose2D(STARTPOS_BLUE_CENTER_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D STARTPOS_BLUE_FAR         =
            new TrcPose2D(STARTPOS_BLUE_FAR_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D[] startPoses              =
            {STARTPOS_BLUE_PROCESSOR, STARTPOS_BLUE_CENTER, STARTPOS_BLUE_FAR};
        //
        // Game element locations and dimensions.
        //
        // AprilTag IDs for various locations for red and blue alliances.
        public static final int[] APRILTAG_LEFT_CORAL_STATION   = {1, 13};
        public static final int[] APRILTAG_RIGHT_CORAL_STATION  = {2, 12};
        public static final int[] APRILTAG_PROCESSOR            = {3, 16};
        public static final int[] APRILTAG_RIGHT_BARGE          = {4, 15};
        public static final int[] APRILTAG_LEFT_BARGE           = {5, 14};
        public static final int[] APRILTAG_CLOSE_LEFT_REEF      = {6, 19};
        public static final int[] APRILTAG_CLOSE_MID_REEF       = {7, 18};
        public static final int[] APRILTAG_CLOSE_RIGHT_REEF     = {8, 17};
        public static final int[] APRILTAG_FAR_RIGHT_REEF       = {9, 22};
        public static final int[] APRILTAG_FAR_MID_REEF         = {10, 21};
        public static final int[] APRILTAG_FAR_LEFT_REEF        = {11, 20};
        public static final int[] APRILTAG_REEFS                = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
        public static final int[][] APRILTAG_STATION            = {{2, 12}, {1, 13}};
        public static final int[] APRILTAG_ALL_STATIONS         = {2, 12, 1, 13};

        // Array of AprilTag poses indexed by AprilTag ID.
        public static final TrcPose2D[] APRILTAG_POSES          =
        {
        /*ID01*/    new TrcPose2D(-25.98, 657.48, -126.0), //z=58.5
        /*ID02*/    new TrcPose2D(-291.34, 657.48, 126.0), //z=58.5
        /*ID03*/    new TrcPose2D(-317.32, 455.12, 90.0), //z=51.125
        /*ID04*/    new TrcPose2D(-241.73, 365.35, 0.0), //z=73.5466,pitch=30
        /*ID05*/    new TrcPose2D(-75.2, 365.35, 0.0), //z=73.5466,pitch=30
        /*ID06*/    new TrcPose2D(-130.32, 530.32, 60.0), //z=12.125
        /*ID07*/    new TrcPose2D(-158.66, 546.85, 0.0), //z=12.125
        /*ID08*/    new TrcPose2D(-187.01, 530.32, -60.0), //z=12.125
        /*ID09*/    new TrcPose2D(-187.01, 497.64, -120.0), //z=12.125
        /*ID10*/    new TrcPose2D(-158.66, 481.5, 180.0), //z=12.125
        /*ID11*/    new TrcPose2D(-130.32, 497.64, 120.0), //z=12.125
        /*ID12*/    new TrcPose2D(-25.98, 33.46, -54.0), //z=58.5
        /*ID13*/    new TrcPose2D(-291.34, 33.46, 54.0), //z=58.5
        /*ID14*/    new TrcPose2D(-241.73, 325.59, 180.0), //z=73.5466,pitch=30
        /*ID15*/    new TrcPose2D(-75.2, 325.59, 180.0), //z=73.5466,pitch=30
        /*ID16*/    new TrcPose2D(0.0, 235.83, -90.0), //z=51.125
        /*ID17*/    new TrcPose2D(-130.32, 160.24, 120.0), //z=12.125
        /*ID18*/    new TrcPose2D(-158.66, 144.09, 180.0), //z=12.125
        /*ID19*/    new TrcPose2D(-187.01, 160.24, -120.0), //z=12.125
        /*ID20*/    new TrcPose2D(-187.01, 192.91, -60.0), //z=12.125
        /*ID21*/    new TrcPose2D(-158.66, 209.45, 0.0), //z=12.125
        /*ID22*/    new TrcPose2D(-130.32, 192.91, 60.0) //z=12.125
        };

        public static final TrcPose2D PROCESSOR_SIDE_LOOKOUT_BLUE =
            new TrcPose2D(-87.0, 150.0, -54.0);
        public static final TrcPose2D FAR_SIDE_LOOKOUT_BLUE =
            new TrcPose2D(-230.0, 150.0, 126.0);
    }   //class Game

    /**
     * This class contains field dimension constants. Generally, these should not change. But some seasons may have
     * slight variations of the field dimensions.
     */
    public static class Field
    {
        public static final boolean mirroredField               = false;
        // Field dimensions in inches.
        public static final double LENGTH                       = 57.0*12.0 + 6.875;    // 690.875
        public static final double WIDTH                        = 26.0*12.0 + 5.0;      // 317.0
    }   //class Field

}   //class RobotParams
