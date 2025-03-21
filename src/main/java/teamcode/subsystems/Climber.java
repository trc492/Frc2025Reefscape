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
import trclib.controller.TrcPidController;
import trclib.motor.TrcMotor;
import trclib.robotcore.TrcEvent;
import trclib.subsystem.TrcMotorGrabber;
import trclib.subsystem.TrcSubsystem;

/**
 * This class implements the Climber Subsystem that consists of an arm and a grabber.
 */
public class Climber extends TrcSubsystem
{
    public static final String SUBSYSTEM_NAME                   = "Climber";
    public static final boolean NEED_ZERO_CAL                   = false;

    public static final class ArmParams
    {
        public static final String COMPONENT_NAME               = SUBSYSTEM_NAME + "Arm";

        public static final String MOTOR_NAME                   = COMPONENT_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_CLIMBERARM_MOTOR;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonFx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = false;

        public static final double DEG_PER_COUNT                = 360.0 / 4096.0;
        public static final double POS_OFFSET                   = 0.0;
        public static final double ZERO_OFFSET                  = 1942.0;   //encoder reading at 0-deg
        public static final double POWER_LIMIT                  = 0.5;

        public static final double MIN_POS                      = 45.0;
        public static final double MAX_POS                      = 200.0;
        public static final double TURTLE_POS                   = 90.0;
        public static final double TURTLE_DELAY                 = 0.0;
        public static final double DEPLOY_POS                   = 180.0;
        public static final double CLIMB_POS                    = 90.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.035, 0.0, 0.001, 0.0, 0.0);
        public static final double POS_PID_TOLERANCE            = 0.5;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.075;
    }   //class ArmParams

    public static final class GrabberParams
    {
        public static final String COMPONENT_NAME               = SUBSYSTEM_NAME + "Grabber";

        public static final String MOTOR_NAME                   = COMPONENT_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_CLIMBERGRABBER_MOTOR;
        public static final MotorType MOTOR_TYPE                = FrcMotorActuator.MotorType.CanTalonFx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = true;

        public static final String SENSOR_NAME                  = COMPONENT_NAME + ".sensor";
        public static final int SENSOR_CHANNEL                  = RobotParams.HwConfig.DIO_CLIMBER_GRABBER_SENSOR;
        public static final boolean SENSOR_TRIGGER_INVERTED     = false;

        public static final double INTAKE_POWER                 = 1.0;
        public static final double EJECT_POWER                  = 0.0;
        public static final double RETAIN_POWER                 = 0.0;
    }   //class GrabberParams

    private static final String DBKEY_ARM_POWER                 = ArmParams.COMPONENT_NAME + "/Power";
    private static final String DBKEY_ARM_CURRENT               = ArmParams.COMPONENT_NAME + "/Current";
    private static final String DBKEY_ARM_POSITION              = ArmParams.COMPONENT_NAME + "/Position";
    private static final String DBKEY_GRABBER_POWER             = GrabberParams.COMPONENT_NAME + "/Power";
    private static final String DBKEY_GRABBER_CURRENT           = GrabberParams.COMPONENT_NAME + "/Current";
    private static final String DBKEY_GRABBER_SENSOR_STATE      = GrabberParams.COMPONENT_NAME + "/SensorState";
    private static final String DBKEY_GRABBER_HAS_OBJECT        = GrabberParams.COMPONENT_NAME + "/HasObject";

    private final FrcDashboard dashboard;
    public final TrcMotor armMotor;
    public final TrcMotorGrabber grabber;

    /**
     * Constructor: Creates an instance of the object.
     */
    public Climber()
    {
        super(SUBSYSTEM_NAME, NEED_ZERO_CAL);
        // Climber Arm.
        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_ARM_POWER, 0.0);
        dashboard.refreshKey(DBKEY_ARM_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_ARM_POSITION, "");
        dashboard.refreshKey(DBKEY_GRABBER_POWER, 0.0);
        dashboard.refreshKey(DBKEY_GRABBER_CURRENT, 0.0);
        dashboard.refreshKey(DBKEY_GRABBER_SENSOR_STATE, false);
        dashboard.refreshKey(DBKEY_GRABBER_HAS_OBJECT, false);

        FrcMotorActuator.Params armMotorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                ArmParams.MOTOR_NAME, ArmParams.MOTOR_ID, ArmParams.MOTOR_TYPE, ArmParams.MOTOR_BRUSHLESS,
                ArmParams.MOTOR_ENC_ABS, ArmParams.MOTOR_INVERTED)
            .setPositionScaleAndOffset(ArmParams.DEG_PER_COUNT, ArmParams.POS_OFFSET, ArmParams.ZERO_OFFSET);
        armMotor = new FrcMotorActuator(armMotorParams).getMotor();
        // Configure encoder.
        // FrcCANTalonSRX talonSrx = (FrcCANTalonSRX) armMotor;
        // talonSrx.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Absolute);
        armMotor.setPositionPidParameters(
            ArmParams.posPidCoeffs, ArmParams.POS_PID_TOLERANCE, ArmParams.SOFTWARE_PID_ENABLED);
        armMotor.setPositionPidPowerComp(this::getArmGravityComp);
        // armMotor.tracer.setTraceLevel(MsgLevel.DEBUG);

        // Climber Grabber.
        FrcMotorGrabber.Params grabberParams = new FrcMotorGrabber.Params()
            .setPrimaryMotor(
                GrabberParams.MOTOR_NAME, GrabberParams.MOTOR_ID, GrabberParams.MOTOR_TYPE,
                GrabberParams.MOTOR_BRUSHLESS, GrabberParams.MOTOR_ENC_ABS, GrabberParams.MOTOR_INVERTED)
            .setDigitalInputTrigger(
                GrabberParams.SENSOR_NAME, GrabberParams.SENSOR_CHANNEL, GrabberParams.SENSOR_TRIGGER_INVERTED)
            .setPowerParams(GrabberParams.INTAKE_POWER, GrabberParams.EJECT_POWER, GrabberParams.RETAIN_POWER);
        grabber = new FrcMotorGrabber(GrabberParams.COMPONENT_NAME, grabberParams).getGrabber();
    }   //Climber

    private double getArmGravityComp(double currPower)
    {
        return ArmParams.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(armMotor.getPosition()));
    }   //getArmGravityComp

    public void deploy(String owner)
    {
        armMotor.setPosition(owner, 0.0, ArmParams.DEPLOY_POS, true, ArmParams.POWER_LIMIT, null, 0.0);
        grabber.autoIntake(owner);
    }   //deploy

    public void climb(String owner)
    {
        armMotor.setPosition(owner, 0.0, ArmParams.CLIMB_POS, true, ArmParams.POWER_LIMIT, null, 0.0);
    }   //climb

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        armMotor.cancel();
        grabber.cancel();
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
        armMotor.setPosition(
            ArmParams.TURTLE_DELAY, ArmParams.TURTLE_POS, true, ArmParams.POWER_LIMIT);
        grabber.cancel();
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
        // Climber Arm.
        dashboard.putNumber(DBKEY_ARM_POWER, armMotor.getPower());
        dashboard.putNumber(DBKEY_ARM_CURRENT, armMotor.getCurrent());
        dashboard.putString(
            DBKEY_ARM_POSITION,
            String.format("%.1f/%.1f", armMotor.getPosition(), armMotor.getPidTarget()));
        // Climber Grabber.
        dashboard.putNumber(DBKEY_GRABBER_POWER, grabber.getPower());
        dashboard.putNumber(DBKEY_GRABBER_CURRENT, grabber.getCurrent());
        dashboard.putBoolean(DBKEY_GRABBER_SENSOR_STATE, grabber.getSensorState());
        dashboard.putBoolean(DBKEY_GRABBER_HAS_OBJECT, grabber.hasObject());

        return lineNum;
    }   //updateStatus

}   //class Climber
