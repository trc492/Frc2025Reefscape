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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import teamcode.subsystems.RobotBase.RobotType;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcGameController.DriveMode;
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
        public static final RobotType robotType                 = RobotType.SwerveRobot;
        public static final boolean inCompetition               = false;
        public static final boolean hybridMode                  = false;
        public static final boolean useTraceLog                 = true;
        public static final boolean useCommStatusMonitor        = true;
        // Status Update
        public static final boolean doStatusUpdate              = true;
        public static final boolean showLoopTime                = false;
        public static final boolean showPowerConsumption        = false;
        public static final boolean showDriveBase               = false;
        public static final boolean showPurePursuitDrive        = false;
        public static final boolean showPidDrive                = false;
        public static final boolean showVision                  = false;
        public static final boolean showSubsystems              = true;
        // Inputs
        public static final boolean useDriverXboxController     = true;
        public static final boolean useOperatorXboxController   = true;
        public static final boolean useTankDrive                = false;
        public static final boolean doOneStickDrive             = false;
        public static final boolean useButtonPanels             = false;
        // Sensors
        public static final boolean useNavX                     = true;
        public static final boolean usePdp                      = false;
        public static final boolean usePressureSensor           = false;
        // Vision
        public static final boolean useVision                   = true;
        public static final boolean usePhotonVision             = true;
        public static final boolean useOpenCvVision             = false;
        public static final boolean useStreamCamera             = false;
        // Drive Base
        public static final boolean useDriveBase                = false;
        public static final boolean useVelocityControl          = false;
        public static final boolean useGyroAssist               = false;
        public static final boolean useAntiTipping              = false;
        public static final boolean useSteeringCANCoder         = false;
        public static final boolean useSteeringCanandmag        = true;
        public static final boolean useSteeringAnalogEncoder    = false;
        // Subsystems
        public static final boolean useSubsystems               = true;
    }   //class Preferences

    /**
     * This class contains the Robot Hardware Configurations.
     */
    public static class HwConfig
    {
        // Joystick ports.
        public static final int XBOX_DRIVER_CONTROLLER          = 0;
        public static final int JSPORT_DRIVER_LEFTSTICK         = 0;
        public static final int JSPORT_DRIVER_RIGHTSTICK        = 1;
        public static final int XBOX_OPERATOR_CONTROLLER        = 2;
        public static final int JSPORT_OPERATORSTICK            = 2;
        public static final int JSPORT_BUTTON_PANEL             = 3;
        public static final int JSPORT_SWITCH_PANEL             = 4;
        // CAN IDs.
        // Drive Motor CAN IDs.
        public static final int CANID_LFDRIVE_MOTOR             = 3;
        public static final int CANID_RFDRIVE_MOTOR             = 4;
        public static final int CANID_LBDRIVE_MOTOR             = 5;
        public static final int CANID_RBDRIVE_MOTOR             = 6;
        // Swerve CAN IDs.
        public static final int CANID_LFSTEER_MOTOR             = 13;
        public static final int CANID_RFSTEER_MOTOR             = 14;
        public static final int CANID_LBSTEER_MOTOR             = 15;
        public static final int CANID_RBSTEER_MOTOR             = 16;
        public static final int CANID_LFSTEER_ENCODER           = 23;
        public static final int CANID_RFSTEER_ENCODER           = 24;
        public static final int CANID_LBSTEER_ENCODER           = 25;
        public static final int CANID_RBSTEER_ENCODER           = 26;
        // Miscellaneous CAN IDs.
        public static final int CANID_PCM                       = 30;
        public static final int CANID_PDP                       = 31;
        // Subsystem CAN IDs.

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
        // Analog Input ports (not used).
        public static final int AIN_ULTRASONIC                  = 0;
        public static final int AIN_PRESSURE_SENSOR             = 0;
        public static final int AIN_LFSTEER_ENCODER             = 0;
        public static final int AIN_RFSTEER_ENCODER             = 1;
        public static final int AIN_LBSTEER_ENCODER             = 2;
        public static final int AIN_RBSTEER_ENCODER             = 3;
        // Digital Input/Output ports.

        // PWM channels.
        public static final int NUM_LEDS                        = 60;
        public static final int PWM_CHANNEL_LED                 = 9;
        // Relay channels.

        // Pneumatic channels.

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
        public static final String ROBOT_CODEBASE               = "Robot2025";
        public static final double ROBOT_LENGTH                 = 34.5;
        public static final double ROBOT_WIDTH                  = 34.5;
        // Robot Drive Parameters.
        public static final DriveMode DRIVE_MODE                = DriveMode.ArcadeMode;
        public static final DriveOrientation DRIVE_ORIENTATION  = DriveOrientation.ROBOT;
        public static final double DRIVE_SLOW_SCALE             = 0.3;
        public static final double DRIVE_NORMAL_SCALE           = 1.0;
        public static final double TURN_SLOW_SCALE              = 0.3;
        public static final double TURN_NORMAL_SCALE            = 0.6;
        public static final double DRIVE_RAMP_RATE              = 0.25;
    }   //class Robot

    public static class Vision
    {
        public static final double ONTARGET_THRESHOLD           = 5.0;
        public static final double GUIDANCE_ERROR_THRESHOLD     = 12.0;
    }   //class Vision

    /**
     * This class contains season specific game element information.
     */
    public static class Game
    {
        public static final boolean fieldIsMirrored             = false;
        //
        // Robot starting positions.
        //
        public static final double STARTPOS_BLUE_Y              = Robot.ROBOT_LENGTH / 2.0;
        public static final double STARTPOS_RED_Y               = Field.LENGTH - STARTPOS_BLUE_Y;
        public static final double STARTPOS_1_X                 = -42.19;
        public static final double STARTPOS_2_X                 = -108.19;
        public static final double STARTPOS_3_X                 = -174.19;
        public static final TrcPose2D STARTPOS_BLUE_1           = new TrcPose2D(STARTPOS_1_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D STARTPOS_BLUE_2           = new TrcPose2D(STARTPOS_2_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D STARTPOS_BLUE_3           = new TrcPose2D(STARTPOS_3_X, STARTPOS_BLUE_Y, 180.0);
        public static final TrcPose2D STARTPOS_RED_1            = new TrcPose2D(STARTPOS_1_X, STARTPOS_RED_Y, 0.0);
        public static final TrcPose2D STARTPOS_RED_2            = new TrcPose2D(STARTPOS_2_X, STARTPOS_RED_Y, 0.0);
        public static final TrcPose2D STARTPOS_RED_3            = new TrcPose2D(STARTPOS_3_X, STARTPOS_RED_Y, 0.0);
        public static final TrcPose2D[][] startPoses            =
        {
            {STARTPOS_BLUE_1, STARTPOS_BLUE_2, STARTPOS_BLUE_3},
            {STARTPOS_RED_1, STARTPOS_RED_2, STARTPOS_RED_3}
        };
        //
        // Game element locations and dimensions.
        //
    }   //class Game

    /**
     * This class contains field dimension constants. Generally, these should not change. But some seasons may have
     * slight variations of the field dimensions.
     */
    public static class Field
    {
        // Field dimensions in inches.
        public static final double LENGTH                       = 54.0*12.0;
        public static final double WIDTH                        = 27.0*12.0;
    }   //class Field

}   //class RobotParams
