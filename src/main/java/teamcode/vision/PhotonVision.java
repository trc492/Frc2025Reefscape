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

package teamcode.vision;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frclib.driverio.FrcDashboard;
import frclib.vision.FrcPhotonVision;
import teamcode.RobotParams;
import teamcode.subsystems.LEDIndicator;
import trclib.pathdrive.TrcPose2D;
import trclib.robotcore.TrcEvent;

/**
 * This class is a thin wrapper extending FrcPhotonVision that provides additional game specific functionalities.
 */
public class PhotonVision extends FrcPhotonVision
{
    private static final String DBKEY_PREFIX                = "Vision/";
    public static final double ONTARGET_THRESHOLD           = 2.0;
    public static final double GUIDANCE_ERROR_THRESHOLD     = 12.0;

    public enum PipelineType
    {
        NONE(0),
        APRILTAG(1),
        ALGAEANDCORAL(2);

        public int pipelineIndex;

        PipelineType(int value)
        {
            pipelineIndex = value;
        }

    }   //enum PipelineType

    private final FrcDashboard dashboard;
    private final String instanceName;
    private final Transform3d robotToCam;
    private final LEDIndicator ledIndicator;
    private PipelineType currPipeline = PipelineType.APRILTAG;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param cameraName specifies the network table name that PhotonVision is broadcasting information over.
     * @param robotToCam specifies the 3D transform location of the camera from robot center.
     * @param ledIndicator specifies the LEDIndicator object, can be null if none provided.
     */
    public PhotonVision(String cameraName, Transform3d robotToCam, LEDIndicator ledIndicator)
    {
        super(cameraName, robotToCam);
        dashboard = FrcDashboard.getInstance();
        dashboard.refreshKey(DBKEY_PREFIX + cameraName, "");
        this.instanceName = cameraName;
        this.robotToCam = robotToCam;
        this.ledIndicator = ledIndicator;
        setPipeline(currPipeline);
    }   //PhotonVision

    /**
     * This method returns the transform between two adjacent AprilTags.
     *
     * @param fromAprilTagId specifies the From AprilTag ID.
     * @param toAprilTagId specifies the To AprilTag ID.
     * @return transform between two adjacent AprilTags.
     */
    public Transform3d getMultiTagTransform(int fromAprilTagId, int toAprilTagId)
    {
        return FrcPhotonVision.getAprilTagFieldPose3d(toAprilTagId, null).minus(
               FrcPhotonVision.getAprilTagFieldPose3d(fromAprilTagId, null));
    }   //getMultiTagTransform

    /**
     * This method returns the robot's field position.
     *
     * @param aprilTagObj specifies the detected AprilTag object.
     * @param usePoseEstimator specifies true to use PhotonVision Lib pose estimator, false to use the AprilTag field
     *        pose to calculate it ourselves.
     * @return robot's field location.
     */
    public TrcPose2D getRobotFieldPose(DetectedObject aprilTagObj, boolean usePoseEstimator)
    {
        return usePoseEstimator? getRobotEstimatedPose(robotToCam):
                                 getRobotPoseFromAprilTagFieldPose(
                                    FrcPhotonVision.getAprilTagFieldPose3d(aprilTagObj.target.getFiducialId(), null),
                                    aprilTagObj.target.getBestCameraToTarget(),
                                    robotToCam);
    }   //getRobotFieldPose

    /**
     * This method returns the best detected object.
     *
     * @param detectionEvent specifies the event to signal when it detects the target.
     * @return best detected object.
     */
    public DetectedObject getBestDetectedObject(TrcEvent detectionEvent)
    {
        DetectedObject bestDetectedObj = super.getBestDetectedObject();

        if (bestDetectedObj != null)
        {
            if (detectionEvent != null)
            {
                detectionEvent.signal();
            }

            if (ledIndicator != null)
            {
                ledIndicator.setPhotonDetectedObject(getPipeline(), bestDetectedObj.targetPose);
            }
        }

        return bestDetectedObj;
    }   //getBestDetectedObject

    /**
     * This method returns the best detected object.
     *
     * @return best detected object.
     */
    @Override
    public DetectedObject getBestDetectedObject()
    {
        return getBestDetectedObject(null);
    }   //getBestDetectedObject

    /**
     * This method finds a matching AprilTag ID in the specified array and returns the found index.
     *
     * @param id specifies the AprilTag ID to be matched.
     * @param aprilTagIds specifies the AprilTag ID array to find the given ID.
     * @return index in the array that matched the ID, -1 if not found.
     */
    private int matchAprilTagId(int id, int[] aprilTagIds)
    {
        int matchedIndex = -1;

        for (int i = 0; i < aprilTagIds.length; i++)
        {
            if (id == aprilTagIds[i])
            {
                matchedIndex = i;
                break;
            }
        }

        return matchedIndex;
    }   //matchAprilTagId

    /**
     * This method get the best detected AprilTag matching the specified AprilTag IDs array sorted by most preferred
     * ID at the top.
     *
     * @param detectionEvent specifies the event to signal when it detects the target.
     * @param aprilTagIds specifies the AprilTag IDs to look for, null to look for any.
     * @return best detected AprilTag.
     */
    public DetectedObject getBestDetectedAprilTag(TrcEvent detectionEvent, int... aprilTagIds)
    {
        DetectedObject bestObj = null;

        if (currPipeline == PipelineType.APRILTAG)
        {
            DetectedObject objects[] = super.getDetectedObjects();

            if (objects != null)
            {
                if (aprilTagIds == null)
                {
                    // Caller did not provide AprilTag IDs to look for, just pick the first one.
                    bestObj = objects[0];
                }
                else
                {
                    int bestIdIndex = -1;
                    for (DetectedObject obj: objects)
                    {
                        int id = obj.target.getFiducialId();
                        int idIndex = matchAprilTagId(id, aprilTagIds);

                        if (idIndex != -1 && (bestIdIndex == -1 || idIndex < bestIdIndex))
                        {
                            // Found first match or a better match.
                            bestObj = obj;
                            bestIdIndex = idIndex;
                        }
                    }
                }
            }
        }

        if (bestObj != null)
        {
            if (detectionEvent != null)
            {
                detectionEvent.signal();
            }

            if  (ledIndicator != null)
            {
                ledIndicator.setPhotonDetectedObject(currPipeline, bestObj.targetPose);
            }
        }

        return bestObj;
    }   //getBestDetectedAprilTag

    /**
     * This method get the best detected AprilTag matching the specified AprilTag IDs array sorted by most preferred
     * ID at the top.
     *
     * @param aprilTagIds specifies the AprilTag IDs to look for, null to look for any.
     * @return best detected AprilTag.
     */
    public DetectedObject getBestDetectedAprilTag(int... aprilTagIds)
    {
        return getBestDetectedAprilTag(null, aprilTagIds);
    }   //getBestDetectedAprilTag

    /**
     * This method sets the active pipeline type used in the LimeLight.
     *
     * @param pipelineType specifies the pipeline to activate in the LimeLight.
     */
    public void setPipeline(PipelineType pipelineType)
    {
        if (pipelineType != currPipeline)
        {
            currPipeline = pipelineType;
            super.setPipelineIndex(pipelineType.pipelineIndex);
        }
    }   //setPipeline

    /**
     * This method returns the active pipeline of the LimeLight.
     *
     * @return active pipeline.
     */
    public PipelineType getPipeline()
    {
        currPipeline = PipelineType.values()[getPipelineIndex()];
        return currPipeline;
    }   //getPipeline

    public static TrcPose2D getClosestAprilTagPose(TrcPose2D robotPose)
    {
        TrcPose2D closestAprilTagPose = null;
        double minDistance = Double.MAX_VALUE;

        for (TrcPose2D aprilTagPose: RobotParams.Game.APRILTAG_POSES)
        {
            double distance = robotPose.distanceTo(aprilTagPose);

            if (distance < minDistance)
            {
                minDistance = distance;
                closestAprilTagPose = aprilTagPose;
            }
        }

        return closestAprilTagPose.clone();
    }   //getClosestAprilTagPose

    //
    // Implements FrcPhotonVision abstract methods.
    //

    /**
     * This method returns the ground offset of the detected target.
     *
     * @return target ground offset.
     */
    public double getTargetGroundOffset(PhotonTrackedTarget target)
    {
        double targetHeight = 0.0;
        PipelineType pipelineType = getPipeline();

        switch (pipelineType)
        {
            case APRILTAG:
                if (target != null)
                {
                    // Even though PhotonVision said detected target, FieldLayout may not give us AprilTagPose.
                    // Check it before access the AprilTag pose.
                    Pose3d aprilTagPose = getAprilTagFieldPose3d(target.getFiducialId(), null);
                    if (aprilTagPose != null)
                    {
                        targetHeight = aprilTagPose.getZ();
                    }
                }
                break;

            default:
                targetHeight = 0.0;
                break;
        }

        return targetHeight;
    }   //getTargetGroundOffset

    /**
     * This method update the dashboard with vision status.
     *
     * @param lineNum specifies the starting line number to print the subsystem status.
     * @return updated line number for the next subsystem to print.
     */
    public int updateStatus(int lineNum)
    {
        PipelineType pipelineType;
        FrcPhotonVision.DetectedObject object = getBestDetectedObject();
        if (object != null)
        {
            pipelineType = getPipeline();
            if (pipelineType == PipelineType.APRILTAG)
            {
                dashboard.putString(
                    "Vision/" + instanceName,
                    String.format("%s[%d]: %s", pipelineType, object.target.getFiducialId(), object));
            }
            else
            {
                dashboard.putString(DBKEY_PREFIX + instanceName, object.toString());
            }
        }
        return lineNum;
    }   //updateStatus

}   //class PhotonVision
