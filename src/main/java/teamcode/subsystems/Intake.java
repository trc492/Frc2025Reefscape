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

import frclib.motor.FrcMotorActuator;
import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcIntake;
import teamcode.RobotParams;
import trclib.motor.TrcMotor;
import trclib.subsystem.TrcIntake;

/**
 * This class implements an Elevator Subsystem.
 */
public class Intake
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Intake";

        public static final String INTAKE_MOTOR_NAME           = SUBSYSTEM_NAME + ".intakeMotor";
        public static final int INTAKE_MOTOR_ID                = RobotParams.HwConfig.CANID_INTAKE_MOTOR;
        public static final MotorType INTAKE_MOTOR_TYPE        = MotorType.CanTalonSrx;
        public static final boolean INTAKE_MOTOR_BRUSHLESS     = false;
        public static final boolean INTAKE_MOTOR_ENC_ABS       = false;
        public static final boolean INTAKE_MOTOR_INVERTED      = false;

        public static final String DEPLOYER_MOTOR_NAME           = SUBSYSTEM_NAME + ".deployerMotor";
        public static final int DEPLOYER_MOTOR_ID                = RobotParams.HwConfig.CANID_INTAKE_MOTOR;
        public static final MotorType DEPLOYER_MOTOR_TYPE        = MotorType.CanTalonSrx;
        public static final boolean DEPLOYER_MOTOR_BRUSHLESS     = false;
        public static final boolean DEPLOYER_MOTOR_ENC_ABS       = false;
        public static final boolean DEPLOYER_MOTOR_INVERTED      = false;

        public static final String DEPLOYER_LOWER_LIMIT_NAME     = SUBSYSTEM_NAME + ".deployerLowerLimit";
        public static final int DEPLOYER_LOWER_LIMIT_CHANNEL     = RobotParams.HwConfig.DIO_DEPLOYER_LOWER_LIMIT;
        public static final boolean DEPLOYER_LOWER_LIMIT_INVERTED = false;

        public static final int SENSOR_DIGITAL_CHANNEL          = 0;
        public static final boolean SENSOR_INVERTED             = false;

        public static final double INTAKE_FORWARD_POWER         = 1.0;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.0;
    }   //class Params

    private final TrcIntake intake;
    private final TrcMotor deployer;
    
    /**
     * Constructor: Creates an instance of the object.
     */
    public Intake()
    {
        FrcIntake.Params intakeParams = new FrcIntake.Params()
            .setPrimaryMotor(
                Params.INTAKE_MOTOR_NAME, Params.INTAKE_MOTOR_ID, Params.INTAKE_MOTOR_TYPE,
                Params.INTAKE_MOTOR_BRUSHLESS, Params.INTAKE_MOTOR_ENC_ABS, Params.INTAKE_MOTOR_INVERTED)
            .setEntryDigitalInput(Params.SENSOR_DIGITAL_CHANNEL, Params.SENSOR_INVERTED, null);
        intake = new FrcIntake(Params.SUBSYSTEM_NAME, intakeParams).getIntake();

        FrcMotorActuator.Params deployerParams = new FrcMotorActuator.Params()
            .setPrimaryMotor(
                Params.DEPLOYER_MOTOR_NAME, Params.DEPLOYER_MOTOR_ID, Params.DEPLOYER_MOTOR_TYPE,
                Params.DEPLOYER_MOTOR_BRUSHLESS, Params.DEPLOYER_MOTOR_ENC_ABS, Params.DEPLOYER_MOTOR_INVERTED)
            .setLowerLimitSwitch(Params.DEPLOYER_LOWER_LIMIT_NAME, Params.DEPLOYER_LOWER_LIMIT_CHANNEL, Params.DEPLOYER_LOWER_LIMIT_INVERTED);
        deployer = new FrcMotorActuator(deployerParams).getMotor();
    }   //Intake

    public TrcIntake getIntake()
    {
        return intake;
    }   //getIntake

    public TrcMotor getDeployer()
    {
        return deployer;
    }   //getIntake

}   //class Intake
