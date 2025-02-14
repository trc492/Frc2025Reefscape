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
 * This class implements the Coral Arm Subsystem.
 */
public class CoralArm extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "CoralArm";

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_CORALARM_MOTOR;
        public static final MotorType MOTOR_TYPE                = MotorType.CanSparkMax;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = false;

        public static final String LOWER_LIMITSWITCH_NAME       = SUBSYSTEM_NAME + ".lowerLimit";
        public static final int LOWER_LIMITSWITCH_CHANNEL       = RobotParams.HwConfig.DIO_CORALARM_LOWER_LIMIT;
        public static final boolean LOWER_LIMITSWITCH_INVERTED  = false;
        public static final String UPPER_LIMITSWITCH_NAME       = SUBSYSTEM_NAME + ".upperLimit";
        public static final int UPPER_LIMITSWITCH_CHANNEL       = RobotParams.HwConfig.DIO_CORALARM_UPPER_LIMIT;
        public static final boolean UPPER_LIMITSWITCH_INVERTED  = false;

        public static final double DEG_PER_COUNT                = 1.0;
        public static final double POS_OFFSET                   = 0.0;
        public static final double POWER_LIMIT                  = 0.5;
        public static final double ZERO_CAL_POWER               = -0.25;

        public static final double MIN_POS                      = POS_OFFSET;
        public static final double MAX_POS                      = 270.0;
        public static final double HOPPER_PICKUP_POS            = 0.0; // TODO
        public static final double[] posPresets                 = {MIN_POS, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0};
        public static final double POS_PRESET_TOLERANCE         = 10.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.018, 0.1, 0.001, 0.0, 2.0);
        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.158;
        public static final double STALL_MIN_POWER              = Math.abs(ZERO_CAL_POWER);
        public static final double STALL_TOLERANCE              = 0.1;
        public static final double STALL_TIMEOUT                = 0.1;
        public static final double STALL_RESET_TIMEOUT          = 0.0;
    }   //class Params

    private final TrcMotor coralArmMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public CoralArm()
    {
        super(Params.SUBSYSTEM_NAME, true);

        FrcMotorActuator.Params motorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_ID, Params.MOTOR_TYPE, Params.MOTOR_BRUSHLESS, Params.MOTOR_ENC_ABS,
                Params.MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.LOWER_LIMITSWITCH_NAME, Params.LOWER_LIMITSWITCH_CHANNEL, Params.LOWER_LIMITSWITCH_INVERTED)
            .setUpperLimitSwitch(Params.UPPER_LIMITSWITCH_NAME, Params.UPPER_LIMITSWITCH_CHANNEL, Params.UPPER_LIMITSWITCH_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        coralArmMotor = new FrcMotorActuator(motorParams).getMotor();
        coralArmMotor.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        coralArmMotor.setPositionPidPowerComp(this::getGravityComp);
        coralArmMotor.setStallProtection(
            Params.STALL_MIN_POWER, Params.STALL_TOLERANCE, Params.STALL_TIMEOUT, Params.STALL_RESET_TIMEOUT);
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
        coralArmMotor.zeroCalibrate(owner, Params.ZERO_CAL_POWER, event);
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
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
        FrcDashboard.getInstance().displayPrintf(
            lineNum++,
            "%s: power=%.3f,pos=%.1f/%.1f,limitSw=%s/%s",
            Params.SUBSYSTEM_NAME, coralArmMotor.getPower(), coralArmMotor.getPosition(), coralArmMotor.getPidTarget(),
            coralArmMotor.isLowerLimitSwitchActive(), coralArmMotor.isUpperLimitSwitchActive());

        return lineNum;
    }   //updateStatus

}   //class CoralArm
