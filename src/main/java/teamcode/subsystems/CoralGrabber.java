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
 * This class implements the Coral Grabber Subsystem.
 */
public class CoralGrabber extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "CoralGrabber";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_CORALGRABBER_MOTOR;
        public static final MotorType MOTOR_TYPE                = FrcMotorActuator.MotorType.CanTalonFx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = true;

        public static final String SENSOR_NAME                  = SUBSYSTEM_NAME + ".sensor";
        public static final int SENSOR_CHANNEL                  = RobotParams.HwConfig.DIO_CORAL_GRABBER_SENSOR;
        public static final boolean SENSOR_TRIGGER_INVERTED     = false;

        public static final double INTAKE_POWER                 = 1.0;
        public static final double EJECT_POWER                  = -0.25;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double DUMP_TIME                    = 0.5;
        public static final double DUMP_DELAY                   = 0.0;
    }   //class Params

    private static final String DBKEY_POWER                     = Params.SUBSYSTEM_NAME + "/Power";
    private static final String DBKEY_CURRENT                   = Params.SUBSYSTEM_NAME + "/Current";
    private static final String DBKEY_SENSOR_STATE              = Params.SUBSYSTEM_NAME + "/SensorState";
    private static final String DBKEY_HAS_OBJECT                = Params.SUBSYSTEM_NAME + "/HasObject";

    private final FrcDashboard dashboard;
    private final TrcMotorGrabber motorGrabber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public CoralGrabber()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_POWER, 0.0);
        dashboard.refreshKey(DBKEY_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_SENSOR_STATE, false);
        dashboard.refreshKey(DBKEY_HAS_OBJECT, false);

        FrcMotorGrabber.Params grabberParams = new FrcMotorGrabber.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_ID, Params.MOTOR_TYPE,Params.MOTOR_BRUSHLESS, Params.MOTOR_ENC_ABS,
                Params.MOTOR_INVERTED)
            .setDigitalInputTrigger(Params.SENSOR_NAME, Params.SENSOR_CHANNEL, Params.SENSOR_TRIGGER_INVERTED)
            .setPowerParams(Params.INTAKE_POWER, Params.EJECT_POWER, Params.RETAIN_POWER);
        motorGrabber = new FrcMotorGrabber(Params.SUBSYSTEM_NAME, grabberParams).getGrabber();
    }   //CoralGrabber

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
        dashboard.putNumber(DBKEY_POWER, motorGrabber.getPower());
        dashboard.putNumber(DBKEY_CURRENT, motorGrabber.getCurrent());
        dashboard.putBoolean(DBKEY_SENSOR_STATE, motorGrabber.getSensorState());
        dashboard.putBoolean(DBKEY_HAS_OBJECT, motorGrabber.hasObject());
        return lineNum;
    }   //updateStatus

}   //class CoralGrabber
