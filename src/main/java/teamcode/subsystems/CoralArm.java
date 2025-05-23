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

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frclib.driverio.FrcDashboard;
import frclib.motor.FrcCANTalonSRX;
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import teamcode.RobotParams;
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements the Coral Arm Subsystem.
 */
public class CoralArm extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "CoralArm";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_CORALARM_MOTOR;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = false;

        public static final boolean LOWER_LIMITSW_NORMAL_CLOSE  = true;
        public static final boolean LOWER_LIMITSW_INVERTED      = true;
        public static final boolean UPPER_LIMITSW_NORMAL_CLOSE  = true;
        public static final boolean UPPER_LIMITSW_INVERTED      = true;

        public static final double DEG_PER_COUNT                = 360.0 / 4096.0;
        public static final double POS_OFFSET                   = 0.0;
        public static final double ZERO_OFFSET                  = 1942.0;   //encoder reading at 0-deg
        public static final double POWER_LIMIT                  = 0.75;

        public static final double MIN_POS                      = -47.0;
        public static final double MAX_POS                      = 180.0;//188.3;
        public static final double TURTLE_POS                   = 15.0;
        public static final double TURTLE_DELAY                 = 0.5;
        public static final double CLIMB_POS                    = 15.0;
        public static final double SAFE_ZONE_POS                = 35.0;
        public static final double STATION_PICKUP_POS           = -43.5;//TODO
        public static final double TROUGH_SCORE_LEVEL_POS       = -43.5;  // TODO
        public static final double REEF_SCORE_LEVEL1_POS        = 115.4;// TODO
        public static final double REEF_SCORE_LEVEL2_POS        = 151.2;// TODO
        public static final double REEF_SCORE_LEVEL3_POS        = 148.0;// TODO
        public static final double[] SCORE_LEVEL_POS            =
            {TROUGH_SCORE_LEVEL_POS, REEF_SCORE_LEVEL1_POS, REEF_SCORE_LEVEL2_POS, REEF_SCORE_LEVEL3_POS};

        public static final double[] posPresets                 = {-30.0, 0.0, 30.0, 60.0, 90.0, 120.0, 150.0};
        public static final double POS_PRESET_TOLERANCE         = 5.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.035, 0.0, 0.001, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.075;
    }   //class Params

    private static final String DBKEY_POWER                     = Params.SUBSYSTEM_NAME + "/Power";
    private static final String DBKEY_CURRENT                   = Params.SUBSYSTEM_NAME + "/Current";
    private static final String DBKEY_POSITION                  = Params.SUBSYSTEM_NAME + "/Position";
    private static final String DBKEY_LOWER_LIMIT_SW            = Params.SUBSYSTEM_NAME + "/LowerLimitSw";
    private static final String DBKEY_UPPER_LIMIT_SW            = Params.SUBSYSTEM_NAME + "/UpperLimitSw";

    private final FrcDashboard dashboard;
    private final TrcMotor coralArmMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public CoralArm()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_POWER, 0.0);
        dashboard.refreshKey(DBKEY_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_POSITION, "");
        dashboard.refreshKey(DBKEY_LOWER_LIMIT_SW, false);
        dashboard.refreshKey(DBKEY_UPPER_LIMIT_SW, false);

        FrcMotorActuator.Params motorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_ID, Params.MOTOR_TYPE, Params.MOTOR_BRUSHLESS, Params.MOTOR_ENC_ABS,
                Params.MOTOR_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET, Params.ZERO_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        coralArmMotor = new FrcMotorActuator(motorParams).getMotor();
        // Configure limit switches
        coralArmMotor.enableLowerLimitSwitch(Params.LOWER_LIMITSW_NORMAL_CLOSE);
        coralArmMotor.setLowerLimitSwitchInverted(Params.LOWER_LIMITSW_INVERTED);
        coralArmMotor.enableUpperLimitSwitch(Params.UPPER_LIMITSW_NORMAL_CLOSE);
        coralArmMotor.setUpperLimitSwitchInverted(Params.UPPER_LIMITSW_INVERTED);
        // Configure encoder.
        FrcCANTalonSRX talonSrx = (FrcCANTalonSRX) coralArmMotor;
        talonSrx.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Absolute);

        coralArmMotor.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        coralArmMotor.setPositionPidPowerComp(this::getGravityComp);
        // coralArmMotor.tracer.setTraceLevel(MsgLevel.DEBUG);
    }   //CoralArm

    public TrcMotor getArmMotor()
    {
        return coralArmMotor;
    }   //getArmMotor

    private double getGravityComp(double currPower)
    {
        return Params.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(coralArmMotor.getPosition()));
    }   //getGravityComp

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        coralArmMotor.cancel();
    }   //cancel

    /**
     * This method starts zero calibrate of the subsystem.
     *
     * @param owner specifies the owner ID to to claim subsystem ownership, can be null if ownership not required.
     * @param event specifies an event to signal when zero calibration is done, can be null if not provided.
     */
    @Override
    public void zeroCalibrate(String owner, TrcEvent event)
    {
        // It has absolute encoder, so don't need zero calibration.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        coralArmMotor.setPosition(Params.TURTLE_DELAY, Params.TURTLE_POS, true, Params.POWER_LIMIT);
    }   //resetState

    /**
     * This method update the dashboard with the subsystem status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    @Override
    public int updateStatus(int lineNum)
    {
        dashboard.putNumber(DBKEY_POWER, coralArmMotor.getPower());
        dashboard.putNumber(DBKEY_CURRENT, coralArmMotor.getCurrent());
        dashboard.putString(
            DBKEY_POSITION, String.format("%.1f/%.1f", coralArmMotor.getPosition(), coralArmMotor.getPidTarget()));
        dashboard.putBoolean(DBKEY_LOWER_LIMIT_SW, coralArmMotor.isLowerLimitSwitchActive());
        dashboard.putBoolean(DBKEY_UPPER_LIMIT_SW, coralArmMotor.isUpperLimitSwitchActive());
        return lineNum;
    }   //updateStatus

}   //class CoralArm
