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
 * This class implements the Winch Subsystem.
 */
public class Winch extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Winch";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_WINCH_MOTOR;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonFx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = false;

        public static final String LOWER_LIMITSWITCH_NAME       = SUBSYSTEM_NAME + ".lowerLimit";
        public static final int LOWER_LIMITSWITCH_CHANNEL       = RobotParams.HwConfig.DIO_WINCH_LOWER_LIMIT;
        public static final boolean LOWER_LIMITSWITCH_INVERTED  = true;

        public static final double GEAR_RATIO                   = 60.0;
        public static final double SPOOL_DIAMETER               = 1.0;
        public static final double INCHES_PER_REV               = 0.0607;   //Math.PI * SPOOL_DIAMETER / GEAR_RATIO;
        public static final double POS_OFFSET                   = 0.0;
        public static final double POWER_LIMIT                  = 1.0;
        public static final double ZERO_CAL_POWER               = -0.7;

        public static final double MIN_POS                      = 0.0;
        public static final double MAX_POS                      = 20.0;
        public static final double DEPLOY_POS                   = 18.0;
        public static final double PRE_CLIMB_POS                = 15.25;
        public static final double CLIMB_POS                    = 3.9;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(1.0, 0.0, 0.0, 0.0, .0);
        public static final double POS_PID_TOLERANCE            = 0.5;
    }   //class Params

    private final TrcMotor winchMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Winch()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        FrcMotorActuator.Params motorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_ID, Params.MOTOR_TYPE, Params.MOTOR_BRUSHLESS, Params.MOTOR_ENC_ABS,
                Params.MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.LOWER_LIMITSWITCH_NAME, Params.LOWER_LIMITSWITCH_CHANNEL, Params.LOWER_LIMITSWITCH_INVERTED)
            .setPositionScaleAndOffset(Params.INCHES_PER_REV, Params.POS_OFFSET);
        winchMotor = new FrcMotorActuator(motorParams).getMotor();
        winchMotor.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        winchMotor.enableLowerLimitSwitch(false);
    }   //Winch

    public TrcMotor getWinchMotor()
    {
        return winchMotor;
    }   //getWinchMotor

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        winchMotor.cancel();
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
        winchMotor.zeroCalibrate(owner, Params.ZERO_CAL_POWER, event);
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
            "%s: power=%.3f,currnet=%.3f,pos=%.1f,limitSw=%s",
            Params.SUBSYSTEM_NAME, winchMotor.getPower(), winchMotor.getCurrent(), winchMotor.getPosition(),
            winchMotor.isLowerLimitSwitchActive());

        return lineNum;
    }   //updateStatus

}   //class Winch
