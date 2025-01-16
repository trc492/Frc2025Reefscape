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

import frclib.motor.FrcMotorActuator.MotorType;
import frclib.subsystem.FrcIntake;
import trclib.subsystem.TrcIntake;

/**
 * This class implements an Elevator Subsystem.
 */
public class Intake
{
    public static final class Params
    {
        public static final String SUBSYSTEM_NAME               = "Intake";
        public static final boolean TWO_MOTOR_INTAKE            = false;

        public static final String PRIMARY_MOTOR_NAME           = SUBSYSTEM_NAME + ".primary";
        public static final int PRIMARY_MOTOR_ID                = 10;
        public static final MotorType PRIMARY_MOTOR_TYPE        = MotorType.CanTalonSrx;
        public static final boolean PRIMARY_MOTOR_BRUSHLESS     = false;
        public static final boolean PRIMARY_MOTOR_ENC_ABS       = false;
        public static final boolean PRIMARY_MOTOR_INVERTED      = !TWO_MOTOR_INTAKE;

        public static final String FOLLOWER_MOTOR_NAME          = SUBSYSTEM_NAME + ".follower";
        public static final int FOLLOWER_MOTOR_ID               = 12;
        public static final MotorType FOLLOWER_MOTOR_TYPE       = MotorType.CanTalonSrx;
        public static final boolean FOLLOWER_MOTOR_BRUSHLESS    = false;
        public static final boolean FOLLOWER_MOTOR_ENC_ABS      = false;
        public static final boolean FOLLOWER_MOTOR_INVERTED     = PRIMARY_MOTOR_INVERTED;

        public static final int SENSOR_DIGITAL_CHANNEL          = 0;
        public static final boolean SENSOR_INVERTED             = false;

        public static final double INTAKE_FORWARD_POWER         = 1.0;
        public static final double RETAIN_POWER                 = 0.0;
        public static final double FINISH_DELAY                 = 0.0;
    }   //class Params

    private final TrcIntake intake;
    
    /**
     * Constructor: Creates an instance of the object.
     */
    public Intake()
    {
        FrcIntake.Params intakeParams = new FrcIntake.Params()
            .setPrimaryMotor(
                Params.PRIMARY_MOTOR_NAME, Params.PRIMARY_MOTOR_ID, Params.PRIMARY_MOTOR_TYPE,
                Params.PRIMARY_MOTOR_BRUSHLESS, Params.PRIMARY_MOTOR_ENC_ABS, Params.PRIMARY_MOTOR_INVERTED)
            .setEntryDigitalInput(Params.SENSOR_DIGITAL_CHANNEL, Params.SENSOR_INVERTED, null);
        if (Params.TWO_MOTOR_INTAKE)
        {
            intakeParams.setFollowerMotor(
                Params.FOLLOWER_MOTOR_NAME, Params.FOLLOWER_MOTOR_ID, Params.FOLLOWER_MOTOR_TYPE,
                Params.FOLLOWER_MOTOR_BRUSHLESS, Params.FOLLOWER_MOTOR_ENC_ABS, Params.FOLLOWER_MOTOR_INVERTED);
        }
        intake = new FrcIntake(Params.SUBSYSTEM_NAME, intakeParams).getIntake();
    }   //Intake

    public TrcIntake getIntake()
    {
        return intake;
    }   //getIntake

}   //class Intake
