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
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcIntake;
import teamcode.RobotParams;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcIntake;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements an Elevator Subsystem.
 */
public class Intake extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Intake";

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_INTAKE_MOTOR;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = false;

        public static final int SENSOR_DIGITAL_CHANNEL          = 0;
        public static final boolean SENSOR_INVERTED             = false;

        public static final double INTAKE_FORWARD_POWER         = 1.0;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.0;

        public static final double coralDistanceThreshold        = 96.0; //TODO: Needs to be adjusted
        public static final double intakePower                   = 1.0;
    }   //class Params

    private final TrcIntake intake;
    
    /**
     * Constructor: Creates an instance of the object.
     */
    public Intake()
    {
        super(Params.SUBSYSTEM_NAME, false);

        FrcIntake.Params intakeParams = new FrcIntake.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_ID, Params.MOTOR_TYPE, Params.MOTOR_BRUSHLESS, Params.MOTOR_ENC_ABS,
                Params.MOTOR_INVERTED)
            .setEntryDigitalInput(Params.SENSOR_DIGITAL_CHANNEL, Params.SENSOR_INVERTED, null);
        intake = new FrcIntake(Params.SUBSYSTEM_NAME, intakeParams).getIntake();
    }   //Intake

    public TrcIntake getIntake()
    {
        return intake;
    }   //getIntake

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        intake.cancel();
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
        // No zero calibration needed.
    }   //zeroCalibrate

    /**
     * This method resets the subsystem state. Typically, this is used to retract the subsystem for turtle mode.
     */
    @Override
    public void resetState()
    {
        // No reset state needed.
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
            "%s: power=%.3f, hasObject=%s, autoActive=%s",
            Params.SUBSYSTEM_NAME, intake.getPower(), intake.hasObject(), intake.isAutoActive());

        return lineNum;
    }   //updateStatus

}   //class Intake
