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

import frclib.driverio.FrcDashboard;
import trclib.subsystem.TrcSubsystem;
import trclib.timer.TrcTimer;

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
        dashboard.refreshKey(DBKEY_PREFERENCE_COMMSTATUS_MONITOR, RobotParams.Preferences.useCommStatusMonitor);
        dashboard.refreshKey(DBKEY_PREFERENCE_UPDATE_DASHBOARD, RobotParams.Preferences.updateDashboard);
        dashboard.refreshKey(DBKEY_PREFERENCE_DRIVEBASE_STATUS, RobotParams.Preferences.showDriveBase);
        dashboard.refreshKey(DBKEY_PREFERENCE_DEBUG_DRIVEBASE, RobotParams.Preferences.debugDriveBase);
        dashboard.refreshKey(DBKEY_PREFERENCE_DEBUG_PIDDRIVE, RobotParams.Preferences.showPidDrive);
        dashboard.refreshKey(DBKEY_PREFERENCE_VISION_STATUS, RobotParams.Preferences.showVision);
        dashboard.refreshKey(DBKEY_PREFERENCE_SUBSYSTEM_STATUS, RobotParams.Preferences.showSubsystems);
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
            if (dashboard.getBoolean(DBKEY_PREFERENCE_UPDATE_DASHBOARD, RobotParams.Preferences.updateDashboard))
            {
                if (dashboard.getBoolean(DBKEY_PREFERENCE_DRIVEBASE_STATUS, RobotParams.Preferences.showDriveBase))
                {
                    lineNum = robot.robotBase.updateStatus(lineNum);
                }

                if (dashboard.getBoolean(DBKEY_PREFERENCE_VISION_STATUS, RobotParams.Preferences.showVision))
                {
                    if (robot.photonVisionFront != null)
                    {
                        lineNum = robot.photonVisionFront.updateStatus(lineNum);
                    }

                    if (robot.photonVisionBack != null)
                    {
                        lineNum = robot.photonVisionBack.updateStatus(lineNum);
                    }
                }

                if (dashboard.getBoolean(DBKEY_PREFERENCE_SUBSYSTEM_STATUS, RobotParams.Preferences.showSubsystems))
                {
                    lineNum = TrcSubsystem.updateStatusAll(lineNum);
                }
            }
        }

        return lineNum;
    }   //updateDashboard

}   //class Dashboard
