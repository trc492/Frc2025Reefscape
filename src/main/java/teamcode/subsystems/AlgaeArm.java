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
 * This class implements the Algae Arm Subsystem.
 */
public class AlgaeArm extends TrcSubsystem
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "AlgaeArm";
        public static final boolean NEED_ZERO_CAL               = false;

        public static final String MOTOR_NAME                   = SUBSYSTEM_NAME + ".motor";
        public static final int MOTOR_ID                        = RobotParams.HwConfig.CANID_ALGAEARM_MOTOR;
        public static final MotorType MOTOR_TYPE                = MotorType.CanTalonSrx;
        public static final boolean MOTOR_BRUSHLESS             = false;
        public static final boolean MOTOR_ENC_ABS               = false;
        public static final boolean MOTOR_INVERTED              = false;

        public static final boolean LOWER_LIMITSW_NORMAL_CLOSE  = true;
        public static final boolean LOWER_LIMITSW_INVERTED      = true;
        public static final boolean UPPER_LIMITSW_NORMAL_CLOSE  = true;
        public static final boolean UPPER_LIMITSW_INVERTED      = true;
        public static final boolean ENCODER_INVERTED            = true;

        public static final double DEG_PER_COUNT                = 360.0 / 4096.0;
        public static final double POS_OFFSET                   = 155.0;
        public static final double ZERO_OFFSET                  = 1768.0;
        public static final double POWER_LIMIT                  = 0.5;

        public static final double MIN_POS                      = 10.0;
        public static final double MAX_POS                      = 150.0;//TODO: tune
        public static final double TURTLE_POS                   = MIN_POS; //TODO: tune
        public static final double CLIMB_POS                    = 15.0;
        public static final double SAFE_ZONE_POS                = MIN_POS; //TODO: tune

        public static final double[] posPresets                 = {MIN_POS, 30.0, 60.0, 90.0, 120.0, 150.0};
        public static final double POS_PRESET_TOLERANCE         = 5.0;

        public static final boolean SOFTWARE_PID_ENABLED        = true;
        public static final TrcPidController.PidCoefficients posPidCoeffs =
            new TrcPidController.PidCoefficients(0.022, 0.0, 0.001, 0.0, 0.0);  //TODO: tune
        public static final double POS_PID_TOLERANCE            = 1.0;
        public static final double GRAVITY_COMP_MAX_POWER       = 0.0;    //TODO: tune
    }   //class Params

    private final TrcMotor algaeArmMotor;

    /**
     * Constructor: Creates an instance of the object.
     */
    public AlgaeArm()
    {
        super(Params.SUBSYSTEM_NAME, Params.NEED_ZERO_CAL);

        FrcMotorActuator.Params motorParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.MOTOR_NAME, Params.MOTOR_ID, Params.MOTOR_TYPE, Params.MOTOR_BRUSHLESS, Params.MOTOR_ENC_ABS,
                Params.MOTOR_INVERTED)
            .setPositionScaleAndOffset(Params.DEG_PER_COUNT, Params.POS_OFFSET)
            .setPositionPresets(Params.POS_PRESET_TOLERANCE, Params.posPresets);
        algaeArmMotor = new FrcMotorActuator(motorParams).getMotor();
        // Configure limit switches
        algaeArmMotor.enableLowerLimitSwitch(Params.LOWER_LIMITSW_NORMAL_CLOSE);
        algaeArmMotor.setLowerLimitSwitchInverted(Params.LOWER_LIMITSW_INVERTED);
        algaeArmMotor.enableUpperLimitSwitch(Params.UPPER_LIMITSW_NORMAL_CLOSE);
        algaeArmMotor.setUpperLimitSwitchInverted(Params.UPPER_LIMITSW_INVERTED);
        // Configure encoder
        FrcCANTalonSRX talonSrx = (FrcCANTalonSRX) algaeArmMotor;
        talonSrx.setFeedbackDevice(FeedbackDevice.CTRE_MagEncoder_Absolute);
        talonSrx.setPositionSensorInverted(Params.ENCODER_INVERTED);

        algaeArmMotor.setPositionPidParameters(Params.posPidCoeffs, Params.POS_PID_TOLERANCE, Params.SOFTWARE_PID_ENABLED);
        algaeArmMotor.setPositionPidPowerComp(this::getGravityComp);
        // algaeArmMotor.tracer.setTraceLevel(MsgLevel.DEBUG);
    }   //AlgaeArm

    public TrcMotor getArmMotor()
    {
        return algaeArmMotor;
    }   //getArmMotor

    private double getGravityComp(double currPower)
    {
        return Params.GRAVITY_COMP_MAX_POWER * Math.sin(Math.toRadians(algaeArmMotor.getPosition()));
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
        algaeArmMotor.cancel();
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
        algaeArmMotor.setPosition(Params.TURTLE_POS);
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
            "%s: power=%.3f,current=%f,pos=%.1f/%.1f,limitSw=%s/%s",
            Params.SUBSYSTEM_NAME, algaeArmMotor.getPower(), algaeArmMotor.getCurrent(), algaeArmMotor.getPosition(),
            algaeArmMotor.getPidTarget(), algaeArmMotor.isLowerLimitSwitchActive(),
            algaeArmMotor.isUpperLimitSwitchActive());

        return lineNum;
    }   //updateStatus

}   //class AlgaeArm
