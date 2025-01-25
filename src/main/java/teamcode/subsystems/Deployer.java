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
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements an Elevator Subsystem.
 */
public class Deployer extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Deployer";

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_DEPLOYER_MOTOR;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = false;

        public static final String LOWER_LIMIT_NAME             = SUBSYSTEM_NAME + ".lowerLimit";
        public static final int LOWER_LIMIT_CHANNEL             = RobotParams.HwConfig.DIO_DEPLOYER_LOWER_LIMIT;
        public static final boolean LOWER_LIMIT_INVERTED        = false;
        public static final double ZERO_CAL_POWER               = -0.25;

        public static final double INTAKE_FORWARD_POWER         = 1.0;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.0;
    }   //class Params

    private final TrcMotor deployer;
    
    /**
     * Constructor: Creates an instance of the object.
     */
    public Deployer()
    {
        super(Params.SUBSYSTEM_NAME, true);

        FrcMotorActuator.Params deployerParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_ID, Params.MOTOR_TYPE, Params.MOTOR_BRUSHLESS, Params.MOTOR_ENC_ABS,
                Params.MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.LOWER_LIMIT_NAME, Params.LOWER_LIMIT_CHANNEL, Params.LOWER_LIMIT_INVERTED);
        deployer = new FrcMotorActuator(deployerParams).getMotor();
    }   //Intake

    public TrcMotor getDeployer()
    {
        return deployer;
    }   //getDeployer

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        deployer.cancel();
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
        deployer.zeroCalibrate(owner, Params.ZERO_CAL_POWER, event);
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
            Params.SUBSYSTEM_NAME, deployer.getPower(), deployer.getPosition(), deployer.getPidTarget(),
            deployer.isLowerLimitSwitchActive(), deployer.isUpperLimitSwitchActive());

        return lineNum;
    }   //updateStatus

}   //class Deployer
