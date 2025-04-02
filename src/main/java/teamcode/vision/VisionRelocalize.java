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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,g
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package teamcode.vision;

import java.util.ArrayList;

import trclib.pathdrive.TrcPose2D;

public class VisionRelocalize
{
    public static class TimedPose
    {
        double timestamp;
        TrcPose2D pose;

        TimedPose(double timestamp, TrcPose2D pose)
        {
            this.timestamp = timestamp;
            this.pose = pose;
        }   //TimedPose
    }   //class TimedPose

    private final ArrayList<TimedPose> timedPoses = new ArrayList<>();

    public void addTimedPose(double timestamp, TrcPose2D pose)
    {
        synchronized (timedPoses)
        {
            timedPoses.add(new TimedPose(timestamp, pose));
        }
    }   //addTimedPose

    public TrcPose2D getRelocalizedPose(double visionTimestamp, TrcPose2D visionPose, TrcPose2D robotPose)
    {
        TrcPose2D relocalizedPose = null;

        synchronized (timedPoses)
        {
            double minTimeDelta = Double.MAX_VALUE;
            TimedPose minTimeDeltaPose = null;
            int minIndex = -1;

            for (int i = 0; i < timedPoses.size(); i++)
            {
                TimedPose timedPose = timedPoses.get(i);
                double timeDelta = Math.abs(visionTimestamp - timedPose.timestamp);
                if (timeDelta < minTimeDelta)
                {
                    minTimeDelta = timeDelta;
                    minTimeDeltaPose = timedPose;
                    minIndex = i;
                }
            }

            TrcPose2D deltaPose = robotPose.relativeTo(minTimeDeltaPose.pose);
            relocalizedPose = visionPose.addRelativePose(deltaPose);
            for (int i = minIndex; i >= 0; i--)
            {
                timedPoses.remove(i);
            }
        }

        return relocalizedPose;
    }   //getRelocalizedPose

}   //class VisionRelocalize
