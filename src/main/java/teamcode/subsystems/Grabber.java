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
import frclib.subsystem.FrcMotorGrabber;
import teamcode.RobotParams;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcMotorGrabber;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements the Grabber Subsystem.
 */
public class Grabber extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Grabber";

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final int PRIMARY_MOTOR_ID                = RobotParams.HwConfig.CANID_GRABBER_MOTOR;
        public static final MotorType PRIMARY_MOTOR_TYPE        = FrcMotorActuator.MotorType.CanTalonSrx;
        public static final boolean PRIMARY_MOTOR_BRUSHLESS     = false;
        public static final boolean PRIMARY_MOTOR_ENC_ABS       = false;
        public static final boolean PRIMARY_MOTOR_INVERTED      = true;

        public static final String SENSOR_NAME                  = SUBSYSTEM_NAME + ".sensor";
        public static final int SENSOR_DIGITAL_CHANNEL          = 0; //TODO; adjust
        public static final boolean SENSOR_TRIGGER_INVERTED     = true; //TODO: adjust

        public static final double INTAKE_POWER                 = 1.0;
        public static final double EJECT_POWER                  = -0.3;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.06;
        public static final double DUMP_TIME                    = 0.5;
        public static final double DUMP_DELAY                   = 0.0;
    }   //class Params

    private final TrcMotorGrabber motorGrabber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Grabber()
    {
        super(Params.SUBSYSTEM_NAME, false);

        FrcMotorGrabber.Params grabberParams = new FrcMotorGrabber.Params()
            .setPrimaryMotor(
                Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_ID, Params.PRIMARY_MOTOR_TYPE,
                Params.PRIMARY_MOTOR_BRUSHLESS, Params.PRIMARY_MOTOR_ENC_ABS, Params.PRIMARY_MOTOR_INVERTED)
            .setPowerParams(Params.INTAKE_POWER, Params.EJECT_POWER, Params.RETAIN_POWER)
            .setDigitalInputTrigger(Params.SENSOR_NAME, Params.SENSOR_DIGITAL_CHANNEL, Params.SENSOR_TRIGGER_INVERTED);
        motorGrabber = new FrcMotorGrabber(Params.SUBSYSTEM_NAME, grabberParams).getGrabber();
    }   //Grabber

    /**
     * This method returns the created MotorGrabber object.
     *
     * @return created grabber object.
     */
    public TrcMotorGrabber getMotorGrabber()
    {
        return motorGrabber;
    }   //getMotorGrabber

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        motorGrabber.cancel();
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
            "%s: power=%.3f, hasObject=%s",
            Params.SUBSYSTEM_NAME, motorGrabber.getPower(), motorGrabber.hasObject());

        return lineNum;
    }   //updateStatus

}   //class Grabber
