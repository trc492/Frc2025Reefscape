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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHEPIXYRWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package teamcode;

import frclib.drivebase.FrcRobotDrive;
import frclib.drivebase.FrcSwerveDrive;
import frclib.driverio.FrcDashboard;
import frclib.vision.FrcPhotonVision;
import teamcode.vision.PhotonVision.PipelineType;
import trclib.controller.TrcPidController;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;
import trclib.vision.TrcOpenCvDetector;
import trclib.vision.TrcVisionTargetInfo;

/**
 * This class contains Dashboard constants and parameters.
 */
public class Dashboard
{
    // Preferences.
    public static final String DBKEY_PREFERENCE_COMMSTATUS_MONITOR  = "Preferences/CommStatusMonitor";
    public static final String DBKEY_PREFERENCE_UPDATE_DASHBOARD    = "Preferences/UpdateDashboard";
    public static final String DBKEY_PREFERENCE_DRIVEBASE_STATUS    = "Preferences/DriveBaseStatus";
    public static final String DBKEY_PREFERENCE_DEBUG_DRIVEBASE     = "Preferences/DebugDriveBase";
    public static final String DBKEY_PREFERENCE_DEBUG_PIDDRIVE      = "Preferences/DebugPidDrive";
    public static final String DBKEY_PREFERENCE_VISION_STATUS       = "Preferences/VisionStatus";
    public static final String DBKEY_PREFERENCE_SUBSYSTEM_STATUS    = "Preferences/SubsystemStatus";
    public static final boolean DEF_COMMSTATUS_MONITOR              = false;
    public static final boolean DEF_UPDATE_DASHBOARD                = false;
    public static final boolean DEF_DRIVEBASE_STATUS                = false;
    public static final boolean DEF_DEBUG_DRIVEBASE                 = false;
    public static final boolean DEF_DEBUG_PIDDRIVE                  = false;
    public static final boolean DEF_VISION_STATUS                   = false;
    public static final boolean DEF_SUBSYSTEM_STATUS                = false;
    // DriveBase.
    public static final String DBKEY_DRIVEBASE_STATE                = "DriveBase/State";
    public static final String DBKEY_DRIVEBASE_DRIVE_ENC            = "DriveBase/DriveEnc";
    public static final String DBKEY_DRIVEBASE_STEER_FRONT          = "DriveBase/SteerFront";
    public static final String DBKEY_DRIVEBASE_STEER_BACK           = "DriveBase/SteerBack";
    // Vision
    public static final String DBKEY_VISION_FRONTCAM                = "Vision/FrontCam";
    public static final String DBKEY_VISION_BACKCAM                 = "Vision/BackCam";
    public static final String DBKEY_VISION_OPENCV                  = "Vision/OpenCv";

    private static FrcDashboard dashboard;
    private static double nextDashboardUpdateTime;

    /**
     * Constructor: Creates an instance of the object and publishes the keys in the Network Table.
     */
    public Dashboard()
    {
        dashboard = FrcDashboard.getInstance();
        nextDashboardUpdateTime = TrcTimer.getCurrentTime();
        // Preferences.
        dashboard.refreshKey(DBKEY_PREFERENCE_COMMSTATUS_MONITOR, DEF_COMMSTATUS_MONITOR);
        dashboard.refreshKey(DBKEY_PREFERENCE_UPDATE_DASHBOARD, DEF_UPDATE_DASHBOARD);
        dashboard.refreshKey(DBKEY_PREFERENCE_DRIVEBASE_STATUS, DEF_DRIVEBASE_STATUS);
        dashboard.refreshKey(DBKEY_PREFERENCE_DEBUG_DRIVEBASE, DEF_DEBUG_DRIVEBASE);
        dashboard.refreshKey(DBKEY_PREFERENCE_DEBUG_PIDDRIVE, DEF_DEBUG_PIDDRIVE);
        dashboard.refreshKey(DBKEY_PREFERENCE_VISION_STATUS, DEF_VISION_STATUS);
        dashboard.refreshKey(DBKEY_PREFERENCE_SUBSYSTEM_STATUS, DEF_SUBSYSTEM_STATUS);
    }   //Dashboard

    /**
     * This method returns the FrcDashboard object.
     *
     * @return dashboard object.
     */
    public FrcDashboard getDashboard()
    {
        return dashboard;
    }   //getDashboard

    /**
     * This method is called periodically to update various hardware/subsystem status of the robot to the dashboard
     * and trace log. In order to lower the potential impact these updates, this method will only update the dashboard
     * at DASHBOARD_UPDATE_INTERVAL.
     *
     * @param robot specifies the robot object.
     * @param lineNum specifies the first Dashboard line for printing status.
     * @return next available dashboard line.
     */
    public static int updateDashboard(Robot robot, int lineNum)
    {
        double currTime = TrcTimer.getCurrentTime();

        if (currTime >= nextDashboardUpdateTime)
        {
            nextDashboardUpdateTime = currTime + RobotParams.Robot.DASHBOARD_UPDATE_INTERVAL;
            if (dashboard.getBoolean(DBKEY_PREFERENCE_UPDATE_DASHBOARD, DEF_UPDATE_DASHBOARD))
            {
                if (dashboard.getBoolean(DBKEY_PREFERENCE_DRIVEBASE_STATUS, DEF_DRIVEBASE_STATUS))
                {
                    if (robot.robotDrive != null)
                    {
                        dashboard.putString(
                            DBKEY_DRIVEBASE_STATE,
                            String.format(
                                "RobotPose=%s, Orient=%s, GyroAssist=%s",
                                robot.robotDrive.driveBase.getFieldPosition(),
                                robot.robotDrive.driveBase.getDriveOrientation(),
                                robot.robotDrive.driveBase.isGyroAssistEnabled()));
                        if (dashboard.getBoolean(DBKEY_PREFERENCE_DEBUG_DRIVEBASE, DEF_DEBUG_DRIVEBASE))
                        {
                            // DriveBase debug info.
                            double lfDriveEnc =
                                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getPosition();
                            double rfDriveEnc =
                                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getPosition();
                            double lbDriveEnc =
                                robot.robotDrive.driveMotors.length > 2?
                                    robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_LEFT_BACK].getPosition(): 0.0;
                            double rbDriveEnc =
                                robot.robotDrive.driveMotors.length > 2?
                                robot.robotDrive.driveMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getPosition(): 0.0;
                            dashboard.putString(
                                DBKEY_DRIVEBASE_DRIVE_ENC,
                                String.format(
                                    "DriveEnc: lf=%.0f, rf=%.0f, lb=%.0f, rb=%.0f, avg=%.0f",
                                    lfDriveEnc, rfDriveEnc, lbDriveEnc, rbDriveEnc,
                                    (lfDriveEnc + rfDriveEnc + lbDriveEnc + rbDriveEnc) /
                                    robot.robotDrive.driveMotors.length));
                            if (robot.robotDrive instanceof FrcSwerveDrive)
                            {
                                FrcSwerveDrive swerveDrive = (FrcSwerveDrive) robot.robotDrive;
                                dashboard.putString(
                                    DBKEY_DRIVEBASE_STEER_FRONT,
                                    String.format(
                                        "angle/motorEnc/absEnc: lf=%.1f/%.3f/%.3f, rf=%.1f/%.3f/%.3f",
                                        swerveDrive.swerveModules[FrcRobotDrive.INDEX_LEFT_FRONT].getSteerAngle(),
                                        swerveDrive.steerMotors[FrcRobotDrive.INDEX_LEFT_FRONT].getMotorPosition(),
                                        swerveDrive.steerEncoders[FrcRobotDrive.INDEX_LEFT_FRONT].getRawPosition(),
                                        swerveDrive.swerveModules[FrcRobotDrive.INDEX_RIGHT_FRONT].getSteerAngle(),
                                        swerveDrive.steerMotors[FrcRobotDrive.INDEX_RIGHT_FRONT].getMotorPosition(),
                                        swerveDrive.steerEncoders[FrcRobotDrive.INDEX_RIGHT_FRONT].getRawPosition()));
                                dashboard.putString(
                                    DBKEY_DRIVEBASE_STEER_BACK,
                                    String.format(
                                        "angle/motorEnc/absEnc: lb=%.1f/%.3f/%.3f, rb=%.1f/%.3f/%.3f",
                                        swerveDrive.swerveModules[FrcRobotDrive.INDEX_LEFT_BACK].getSteerAngle(),
                                        swerveDrive.steerMotors[FrcRobotDrive.INDEX_LEFT_BACK].getMotorPosition(),
                                        swerveDrive.steerEncoders[FrcRobotDrive.INDEX_LEFT_BACK].getRawPosition(),
                                        swerveDrive.swerveModules[FrcRobotDrive.INDEX_RIGHT_BACK].getSteerAngle(),
                                        swerveDrive.steerMotors[FrcRobotDrive.INDEX_RIGHT_BACK].getMotorPosition(),
                                        swerveDrive.steerEncoders[FrcRobotDrive.INDEX_RIGHT_BACK].getRawPosition()));
                            }

                            if (dashboard.getBoolean(DBKEY_PREFERENCE_DEBUG_PIDDRIVE, DEF_DEBUG_PIDDRIVE))
                            {
                                TrcPidController xPidCtrl = robot.robotDrive.pidDrive.getXPidCtrl();
                                if (xPidCtrl != null)
                                {
                                    xPidCtrl.displayPidInfo(lineNum);
                                    lineNum += 2;
                                }
                                robot.robotDrive.pidDrive.getYPidCtrl().displayPidInfo(lineNum);
                                lineNum += 2;
                                robot.robotDrive.pidDrive.getTurnPidCtrl().displayPidInfo(lineNum);
                                lineNum += 2;
                            }
                        }
                    }
                }

                if (dashboard.getBoolean(DBKEY_PREFERENCE_VISION_STATUS, DEF_VISION_STATUS))
                {
                    PipelineType pipelineType;
                    if (robot.photonVisionFront != null)
                    {
                        FrcPhotonVision.DetectedObject object = robot.photonVisionFront.getBestDetectedObject();
                        if (object != null)
                        {
                            pipelineType = robot.photonVisionFront.getPipeline();
                            dashboard.putString(
                                DBKEY_VISION_FRONTCAM,
                                String.format("%s[%d]: %s", pipelineType, object.target.getFiducialId(), object));
                        }
                    }

                    if (robot.photonVisionBack != null)
                    {
                        FrcPhotonVision.DetectedObject object = robot.photonVisionBack.getBestDetectedObject();
                        if (object != null)
                        {
                            pipelineType = robot.photonVisionFront.getPipeline();
                            dashboard.putString(
                                DBKEY_VISION_BACKCAM,
                                String.format("%s[%d]: %s", pipelineType, object.target.getFiducialId(), object));
                        }
                    }

                    if (robot.openCvVision != null)
                    {
                        TrcVisionTargetInfo<TrcOpenCvDetector.DetectedObject<?>> object =
                            robot.openCvVision.getDetectedTargetInfo(null, null);
                        if (object != null)
                        {
                            dashboard.putString(DBKEY_VISION_OPENCV, String.format("%s", object));
                        }
                    }
                }

                if (dashboard.getBoolean(DBKEY_PREFERENCE_SUBSYSTEM_STATUS, DEF_SUBSYSTEM_STATUS))
                {
                    lineNum = TrcSubsystem.updateStatusAll(lineNum);
                }
            }
        }

        return lineNum;
    }   //updateDashboard

}   //class Dashboard
