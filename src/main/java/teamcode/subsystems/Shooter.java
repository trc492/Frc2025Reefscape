/*
 * Copyright (c) 2024 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
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
import frclib.motor.FrcCANSparkMax;
import frclib.motor.FrcCANTalonFX;
import trclib.dataprocessor.TrcUtil;
import trclib.robotcore.TrcEvent;
import trclib.robotcore.TrcPidController.PidCoefficients;
import trclib.subsystem.TrcShooter;
import trclib.subsystem.TrcSubsystem;

public class Shooter extends TrcSubsystem
{
    private static final String moduleName = Shooter.class.getSimpleName();

    public static class Params
    {
        public static final String SUBSYSTEM_NAME               = "Shooter";
        public static final int shooterCandId                   = 17;
        public static final boolean shooterMotorInverted        = false;
        public static final double shooterGearRatio             = 1.0;
        public static final double shooterPosScale              = 1.0 / shooterGearRatio;   // in rot.
        public static final PidCoefficients shooterVelPidCoeff  = new PidCoefficients(0.38, 0.0, 0.000098, 0.120);
        public static final double shooterVelTolerance          = 3.0;      // in rps.
        public static final double shooterMaxVelocity           = 100.0;    // in rps.
        public static final double shooterMaxAcceleration       = 100.0;    // in rps square.
        public static final double shooterVelMinInc             = 1.0;      // in rps.
        public static final double shooterVelMaxInc             = 10.0;     // in rps.
        public static final double shooterSpeakerCloseVelocity  = 90.0;     // in rps.
        public static final double shooterAmpVelocity           = 17.5;     // in rps.
        public static final double shooterDumpVelocity          = 90.0;     // in rps.
        public static final double shooterSourcePickupVelocity  = -20.0;    // in rps.
        public static final double shooterPresetVelTolerance    = 5.0;      // in rps.
        public static final double[] shooterPresetVelocities    = new double[]
            {20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0};

        public static final int tiltCanId                       = 7;
        public static final boolean tiltMotorInverted           = true;
        public static final double tiltGearRatio                = 59.0/18.0;
        public static final double tiltPosScale                 = 360.0 / tiltGearRatio;
        public static final double tiltPosOffset                = -15.0;    // in degrees
        public static final double tiltZeroOffset               = 0.029;    // in raw encoder unit
        public static final double tiltPowerLimit               = 0.5;
        public static final PidCoefficients tiltPosPidCoeff     = new PidCoefficients(0.023, 0.0, 0.001, 0.0);
        public static final double tiltPosPidTolerance          = 1.0;
        public static final double tiltMinAngle                 = 0.0;
        public static final double tiltMaxAngle                 = 87.0;     // in degrees.
        public static final double tiltAngleMinInc              = 1.0;      // in degrees.
        public static final double tiltAngleMaxInc              = 10.0;     // in degrees.
        public static final double tiltTurtleAngle              = 35.0;     // in degrees.
        public static final double tiltSpeakerFarAngle          = 52.0;     // in degrees.
        public static final double tiltAmpAngle                 = 57.0;     // in degrees.
        public static final double tiltDumpAngle                = 39.0;     // in degrees.
        public static final double tiltSpeakerCloseAngle        = 64.0;     // in degrees.
        public static final double tiltSourcePickupAngle        = 88.0;     // in degrees.

        public static final double tiltPresetPosTolerance       = 1.0;      // in degrees.
        public static final double[] tiltPresetPositions        = new double[]
            {tiltTurtleAngle, tiltSpeakerFarAngle, tiltAmpAngle, tiltSpeakerCloseAngle, tiltSourcePickupAngle};

        // Talked with Jackson and said that we would most likely not score in trap,
        // so I don't think there is a need to tune ... Will leave it just in case.
        // public static final ShootParamTable.Params stageShootParams = new ShootParamTable.Params(
        //     "Stage", 0.0, 30.0, 60.0);

        // public static final String SPEAKER_UPCLOSE_ENTRY        = "Speaker0ft";
        // public static final String WING_NOTE_ENTRY              = "Speaker5ft";
        // // public static final ShootParamTable speakerShootParamTable = new ShootParamTable()
        // //     .add(SPEAKER_UPCLOSE_ENTRY, 55.0, shooterSpeakerCloseVelocity, tiltSpeakerCloseAngle)
        // //     .add("Speaker1ft",          66.9, 90.0, 60.0)   // 57.0
        // //     .add("Speaker2ft",          78.2, 90.0, 55.0)   // 52.0
        // //     .add("Speaker3ft",          90.3, 90.0, 50.0)   // 47.0
        // //     .add("Speaker4ft",          102.0, 90.0, 47.0)  // 44.0
        // //     .add("Speaker5ft",          114.0, 90.0, 44.0)  // 41.0
        // //     .add("Speaker6ft",          125.3, 90.0, 41.0)  // 38.0
        // //     .add("Speaker7ft",          137.3, 90.0, 40.0);
        // public static final ShootParamTable speakerShootParamTable = new ShootParamTable()
        //     .add(SPEAKER_UPCLOSE_ENTRY, 55.0, shooterSpeakerCloseVelocity, tiltSpeakerCloseAngle)
        //     .add("Speaker1ft",          66.9, 90.0, 59.0)   // 57.0
        //     .add("Speaker2ft",          78.2, 90.0, 54.0)   // 52.0
        //     .add("Speaker3ft",          90.3, 90.0, 49.0)   // 47.0
        //     .add("Speaker4ft",          102.0, 90.0, 47.0)  // 44.0
        //     .add("Speaker5ft",          114.0, 90.0, 44.0)  // 41.0
        //     .add("Speaker6ft",          125.3, 90.0, 42.0)  // 38.0
        //     .add("Speaker7ft",          137.3, 90.0, 41.0);
        // public static final ShootParamTable.Params wingNotePresetParams = speakerShootParamTable.get(WING_NOTE_ENTRY);
    }   //class Params

    private static Shooter instance = null;
    private final FrcCANTalonFX shooterMotor;
    private final FrcCANSparkMax tiltMotor;
    private final TrcShooter shooter;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param shootOp specifies the interface to call for shooting the object.
     */
    public Shooter(TrcShooter.ShootOperation shootOp)
    {
        super(Params.SUBSYSTEM_NAME, false);

        shooterMotor = new FrcCANTalonFX(moduleName + ".shooterMotor", Params.shooterCandId);
        shooterMotor.resetFactoryDefault();
        shooterMotor.setMotorInverted(Params.shooterMotorInverted);
        shooterMotor.disableLowerLimitSwitch();
        shooterMotor.disableUpperLimitSwitch();
        shooterMotor.setBrakeModeEnabled(false);
        shooterMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        shooterMotor.enableMotionProfile(
            Params.shooterMaxVelocity, Params.shooterMaxAcceleration, 0.0);
        shooterMotor.setPositionSensorScaleAndOffset(Params.shooterPosScale, 0.0);
        shooterMotor.setVelocityPidParameters(
            Params.shooterVelPidCoeff, Params.shooterVelTolerance);
        shooterMotor.setPresets(
            true, Params.shooterPresetVelTolerance, Params.shooterPresetVelocities);

        tiltMotor = new FrcCANSparkMax(moduleName + ".tiltMotor", Params.tiltCanId, false, true);
        tiltMotor.resetFactoryDefault();
        tiltMotor.setMotorInverted(Params.tiltMotorInverted);
        tiltMotor.setBrakeModeEnabled(true);
        tiltMotor.setVoltageCompensationEnabled(TrcUtil.BATTERY_NOMINAL_VOLTAGE);
        tiltMotor.enableLowerLimitSwitch(true);
        tiltMotor.enableUpperLimitSwitch(true);
        tiltMotor.setPositionSensorScaleAndOffset(Params.tiltPosScale, Params.tiltPosOffset, Params.tiltZeroOffset);
        // tiltMotor.resetPositionOnLowerLimitSwitch();
        // We are using software position PID control for Tilt. So we just enable software PID before setting
        // PID coefficients.
        tiltMotor.setSoftwarePidEnabled(true);
        tiltMotor.setPositionPidParameters(Params.tiltPosPidCoeff, Params.tiltPosPidTolerance);
        // Tilt is heavily geared down, so don't really need gravity compensation.
        // tiltMotor.setPositionPidPowerComp(this::getTiltGravityComp);
        tiltMotor.setPresets(
            false, Params.tiltPresetPosTolerance, Params.tiltPresetPositions);

        TrcShooter.PanTiltParams tiltParams = new TrcShooter.PanTiltParams(
            Params.tiltPowerLimit, Params.tiltMinAngle, Params.tiltMaxAngle);
        shooter = new TrcShooter(moduleName, shooterMotor, null, tiltMotor, tiltParams, null, null);
        instance = this;
    }   //Shooter

    /**
     * This method returns the instance name.
     */
    @Override
    public String toString()
    {
        return shooter.toString();
    }   //toString

    /**
     * This method returns the Shooter parent object instance.
     *
     * @return shooter parent instance.
     */
    public static Shooter getInstance()
    {
        return instance;
    }   //getInstance

    // /**
    //  * This method returns the Pigeon's yaw value.
    //  *
    //  * @return yaw value.
    //  */
    // public static double getTilterYaw()
    // {
    //     double value = 0.0;

    //     if (instance != null && instance.pigeonIMU != null)
    //     {
    //         value = instance.pigeonIMU.getYaw();
    //     }

    //     return value;
    // }   //getTilterYaw

    // /**
    //  * This method returns the Pigeon's pitch value.
    //  *
    //  * @return pitch value.
    //  */
    // public static double getTilterPitch()
    // {
    //     double value = 0.0;

    //     if (instance != null && instance.pigeonIMU != null)
    //     {
    //         value = instance.pigeonIMU.getPitch();
    //     }

    //     return value;
    // }   //getTilterPitch

    // /**
    //  * This method returns the Pigeon's roll value.
    //  *
    //  * @return roll value.
    //  */
    // public static double getTilterRoll()
    // {
    //     double value = 0.0;

    //     if (instance != null && instance.pigeonIMU != null)
    //     {
    //         value = instance.pigeonIMU.getRoll();
    //     }

    //     return value;
    // }   //getTilterRoll

    /**
     * This method clears all tilter motor faults.
     */
    public static void clearTilterFaults()
    {
        if (instance != null)
        {
            instance.tiltMotor.motor.clearFaults();
        }
    }   //clearTilterFaults

    /**
     * This method returns the created TrcShooter object.
     *
     * @return TrcShooter object.
     */
    public TrcShooter getShooter()
    {
        return shooter;
    }   //getShooter

    // /**
    //  * This method is called by PID control to determine the power required to compensate for gravity in essence
    //  * making tilt gravity neutral (i.e. hold its position, aka feedforward).
    //  *
    //  * @param currPower specifies the current tilt power (not used).
    //  * @return gravity compensation power.
    //  */
    // private double getTiltGravityComp(double currPower)
    // {
    //     double gravityComp =
    //         RobotParams.Shooter.tiltMaxHoldingPower * Math.cos(Math.toRadians(shooter.getTiltAngle()));
    //     shooter.tracer.traceDebug(moduleName, "gravityComp=" + gravityComp);
    //     return gravityComp;
    // }   //getTiltGravityComp

    //
    // Implements TrcSubsystem abstract methods.
    //

    /**
     * This method cancels any pending operations.
     */
    @Override
    public void cancel()
    {
        shooter.cancel();
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
        shooter.setTiltAngle(Shooter.Params.tiltTurtleAngle);
        shooter.stopShooter();
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
        FrcDashboard dashboard = FrcDashboard.getInstance();

        dashboard.displayPrintf(
            lineNum++,
            "%s: power=%.3f, vel=%.3f",
            Params.SUBSYSTEM_NAME, shooter.getShooterMotor1Power(), shooter.getShooterMotor1Velocity());
        dashboard.displayPrintf(
            lineNum++,
            "Tilt: power=%.3f, pos=%.1f, limitSw=%s/%s",
            shooter.getTiltPower(), shooter.getTiltAngle(),
            shooter.tiltLowerLimitSwitchActive(), shooter.tiltUpperLimitSwitchActive());
    
        return lineNum;
    }   //updateStatus

}   //class Shooter
