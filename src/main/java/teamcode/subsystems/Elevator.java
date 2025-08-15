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

import frclib.driverio.FrcDashboard;
import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import teamcode.RobotParams;
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements an Elevator Subsystem.
 */
public class Elevator extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Elevator";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_ELEVATOR_MOTOR;
        public static final MotorType MOTOR_TYPE                = MotorType.CanSparkMax;
        public static final boolean MOTOR_BRUSHLESS             = true;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = true;

        public static final boolean LOWER_LIMITSW_NORMAL_CLOSE  = true;
        public static final boolean UPPER_LIMITSW_NORMAL_CLOSE  = true;

        public static final double INCHES_PER_COUNT             = 0.17682926829268292682926829268293;
        public static final double POS_OFFSET                   = 39.25;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.5;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 70.5;
        public static final double TURTLE_POS                   = MIN_POS;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double CLIMB_POS                    = MIN_POS;
        public static final double SAFE_ZONE_POS                = 50.0;
        public static final double STATION_PICKUP_POS           = 43.5;  //TODO
        public static final double TROUGH_SCORE_LEVEL_POS       = MIN_POS;  // TODO
        public static final double REEF_SCORE_LEVEL1_POS        = MIN_POS;  // TODO
        public static final double REEF_SCORE_LEVEL2_POS        = 43.5; // TODO
        public static final double REEF_SCORE_LEVEL3_POS        = 70.0; // TODO
        public static final double[] SCORE_LEVEL_POS            =
            {TROUGH_SCORE_LEVEL_POS, REEF_SCORE_LEVEL1_POS, REEF_SCORE_LEVEL2_POS, REEF_SCORE_LEVEL3_POS};
        public static final double[] posPresets                 = {MIN_POS, 45.0, 50.0, 55.0, 60.0, 65.0, MAX_POS};
        public static final double POS_PRESET_TOLERANCE         = 2.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.5, 0.0, 0.01, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.5;
        public static final double GRAVITY_COMP_POWER           = 0.0;
    }   //class Params

    private static final String DBKEY_POWER                     = Params.SUBSYSTEM_NAME + "/Power";
    private static final String DBKEY_CURRENT                   = Params.SUBSYSTEM_NAME + "/Current";
    private static final String DBKEY_POSITION                  = Params.SUBSYSTEM_NAME + "/Position";
    private static final String DBKEY_LOWER_LIMIT_SW            = Params.SUBSYSTEM_NAME + "/LowerLimitSw";
    private static final String DBKEY_UPPER_LIMIT_SW            = Params.SUBSYSTEM_NAME + "/UpperLimitSw";

    private final FrcDashboard dashboard;
    private final TrcMotor elevatorMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Elevator()
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
            .setPositionScaleAndOffset(Params.INCHES_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);

        elevatorMotor = new FrcMotorActuator(motorParams).getMotor();
        // Configure limit switches
        elevatorMotor.enableLowerLimitSwitch(Params.LOWER_LIMITSW_NORMAL_CLOSE);
        elevatorMotor.enableUpperLimitSwitch(Params.UPPER_LIMITSW_NORMAL_CLOSE);
        elevatorMotor.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        // Looks like mechanical gravity comp works well, so don't need software gravity comp.
        // elevatorMotor.setPositionPidPowerComp(this::getGravityComp);
        // elevatorMotor.tracer.setTraceLevel(MsgLevel.DEBUG);
    }   //Elevator

    public TrcMotor getElevatorMotor()
    {
        return elevatorMotor;
    }   //getElevatorMotor

    // private double getGravityComp(double currPower)
    // {
    //     return Elevator.Params.GRAVITY_COMP_POWER;
    // }   //getGravityComp

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        elevatorMotor.cancel();
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
        elevatorMotor.zeroCalibrate(owner, Params.ZERO_CAL_POWER, event);
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        elevatorMotor.setPosition(Params.TURTLE_DELAY, Params.TURTLE_POS, true, Params.POWER_LIMIT);
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
        dashboard.putNumber(DBKEY_POWER, elevatorMotor.getPower());
        dashboard.putNumber(DBKEY_CURRENT, elevatorMotor.getCurrent());
        dashboard.putString(
            DBKEY_POSITION, String.format("%.1f/%.1f", elevatorMotor.getPosition(), elevatorMotor.getPidTarget()));
        dashboard.putBoolean(DBKEY_LOWER_LIMIT_SW, elevatorMotor.isLowerLimitSwitchActive());
        dashboard.putBoolean(DBKEY_UPPER_LIMIT_SW, elevatorMotor.isUpperLimitSwitchActive());
        return lineNum;
    }   //updateStatus

    /**
     * This method is called to prep the subsystem for tuning.
     *
     * @param tuneParams specifies tuning parameters.
     */
    @Override
    public void prepSubsystemForTuning(double... tuneParams)
    {
    }   //prepSubsystemForTuning

}   //class Elevator
 