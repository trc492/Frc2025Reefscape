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

import frclib.dataprocessor.FrcColor;
import frclib.driverio.FrcAddressableLED;
import teamcode.RobotParams;
import teamcode.vision.PhotonVision;
import trclib.drivebase.TrcDriveBase.DriveOrientation;
import trclib.driverio.TrcAddressableLED;
import trclib.pathdrive.TrcPose2D;

public class LEDIndicator
{
    private static final TrcAddressableLED.Pattern aprilTagLockedPattern =  // Magenta
        new TrcAddressableLED.Pattern("AprilTagLocked", new FrcColor(63, 0, 63), RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern aprilTagPattern =        // Green
        new TrcAddressableLED.Pattern("AprilTag", new FrcColor(0, 63, 0), RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern seeNothingPattern =      // Red
        new TrcAddressableLED.Pattern("SeeNothing", new FrcColor(63, 0, 0), RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern fieldOrientedPattern =   // White
        new TrcAddressableLED.Pattern("FieldOriented", new FrcColor(63, 63, 63), RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern reefLevel4Pattern =      // Yellow
        new TrcAddressableLED.Pattern("ReefLevel4", new FrcColor(63, 63, 0), RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern reefLevel3Pattern =      // Yellow
        new TrcAddressableLED.Pattern("ReefLevel3", new FrcColor(63, 63, 0), RobotParams.HwConfig.NUM_LEDS*3/4);
    private static final TrcAddressableLED.Pattern reefLevel2Pattern =      // Yellow
        new TrcAddressableLED.Pattern("ReefLevel2", new FrcColor(63, 63, 0), RobotParams.HwConfig.NUM_LEDS/2);
    private static final TrcAddressableLED.Pattern reefLevel1Pattern =      // Yellow
        new TrcAddressableLED.Pattern("ReefLevel1", new FrcColor(63, 63, 0), RobotParams.HwConfig.NUM_LEDS/4);
    private static final TrcAddressableLED.Pattern leftReefBranchPattern =  // Red
        new TrcAddressableLED.Pattern("ReefLevel1", new FrcColor(63, 0, 0), RobotParams.HwConfig.NUM_LEDS/2);
    private static final TrcAddressableLED.Pattern rightReefBranchPattern =  // Cyan
        new TrcAddressableLED.Pattern("ReefLevel1", new FrcColor(0, 63, 63), RobotParams.HwConfig.NUM_LEDS/2);
    private static final TrcAddressableLED.Pattern robotOrientedPattern =   // Blue
        new TrcAddressableLED.Pattern("RobotOriented", new FrcColor(0, 0, 63), RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern inverseOrientedPattern = // Magenta
        new TrcAddressableLED.Pattern("InverseOriented", new FrcColor(63, 0, 63), RobotParams.HwConfig.NUM_LEDS);
    private static final TrcAddressableLED.Pattern nominalPattern =         // Black
        new TrcAddressableLED.Pattern("Nominal", new FrcColor(0, 0, 0), RobotParams.HwConfig.NUM_LEDS);

    private static final TrcAddressableLED.Pattern[] priorities =
    {
        // Highest priority
        aprilTagLockedPattern,
        aprilTagPattern,
        seeNothingPattern,
        reefLevel4Pattern,
        reefLevel3Pattern,
        reefLevel2Pattern,
        reefLevel1Pattern,
        fieldOrientedPattern,
        robotOrientedPattern,
        inverseOrientedPattern,
        nominalPattern
        // Lowest priority
    };

    private static final TrcAddressableLED.Pattern[] reefLevelPatterns =
    {
        reefLevel1Pattern,
        reefLevel2Pattern,
        reefLevel3Pattern,
        reefLevel4Pattern
    };

    private FrcAddressableLED led;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param name specifies the LED instance name.
     * @param channel specifies the PWM channel of the LED.
     * @param numLEDs specifies the number of LED pixels.
     */
    public LEDIndicator(String name, int channel, int numLEDs)
    {
        led = new FrcAddressableLED(name, channel, numLEDs);
        reset();
    }   //LEDIndicator

    /**
     * This method resets the LED strip to the nominal pattern.
     */
    public void reset()
    {
        led.setEnabled(true);
        led.setPatternPriorities(priorities);
        led.reset();
        led.resetAllPatternStates();
        led.setPatternState(nominalPattern, true);
    }   //reset

    /**
     * This method sets the LED to indicate the reef level to score the coral.
     *
     * @param level specifies the reef score level (0-3: 0 being trough and 3 being highest level).
     */
    public void setReefLevel(int level)
    {
        led.setPatternState(reefLevelPatterns[level], true, 0.5);
    }   //setReefLevel

    /**
     * This method sets the LED to indicate the left or right reef branch to score.
     *
     * @param rightBranch specifies true to score the right branch, false to score the left branch.
     */
    public void setReefBranch(boolean rightBranch)
    {
        led.setPatternState(rightBranch? rightReefBranchPattern: leftReefBranchPattern, true, 0.5);
    }   //setReefBranch

    /**
     * This method sets the LED to indicate the drive orientation mode of the robot.
     *
     * @param orientation specifies the drive orientation mode.
     */
    public void setDriveOrientation(DriveOrientation orientation)
    {
        switch (orientation)
        {
            case INVERTED:
                led.setPatternState(inverseOrientedPattern, true);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, false);
                break;

            case ROBOT:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, true);
                led.setPatternState(fieldOrientedPattern, false);
                break;

            case FIELD:
                led.setPatternState(inverseOrientedPattern, false);
                led.setPatternState(robotOrientedPattern, false);
                led.setPatternState(fieldOrientedPattern, true);
                break;
        }
    }   //setDriveOrientation

    /**
     * This method sets the LED to indicate the type of Photon Vision detected object.
     *
     * @param pipelineType specifies the detected object type (by its pipeline), null if none detected.
     * @param objPose specifies the detected object pose, valid if pipelineType is not null.
     */
    public void setPhotonDetectedObject(PhotonVision.PipelineType pipelineType, TrcPose2D objPose)
    {
        if (pipelineType == null)
        {
            led.setPatternState(seeNothingPattern, true, 0.5);
        }
        else
        {
            switch (pipelineType)
            {
                case APRILTAG:
                    if (Math.abs(Math.atan(objPose.x / objPose.y)) < PhotonVision.ONTARGET_THRESHOLD)
                    {
                        led.setPatternState(aprilTagLockedPattern, true, 0.5);
                    }
                    else
                    {
                        led.setPatternState(aprilTagPattern, true, 0.5);
                    }
                    break;

                default:
                    break;
            }
        }
    }   //setPhotonDetectedObject

}   //class LEDIndicator
