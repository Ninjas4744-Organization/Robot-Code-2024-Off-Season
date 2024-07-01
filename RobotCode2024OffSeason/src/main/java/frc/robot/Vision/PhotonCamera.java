package frc.robot.Vision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.*;
import frc.robot.FieldConstants;
import frc.robot.Constants.VisionConstants;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCamera {
    private final PhotonCamera _camera;
    private final PhotonPoseEstimator _estimator;
    private List<PhotonTrackedTarget> _targets;
    private VisionIOOutput _output;

    /**
     * Implements PhotonVision camera
     *
     * @param name Name of the camera.
     * @param cameraPose Location of the camera on the robot (from center, positive x forward, positive y left, and positive angle is counterclockwise).
     */
    public PhotonCamera(String name, Transform3d cameraPose) {
        _camera = new PhotonCamera(name);

        _estimator = new PhotonPoseEstimator(FieldConstants.getFieldLayout(), PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, _camera, cameraPose);
        _estimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);

        _output = new VisionIOOutput();
    }

    @Override
    public VisionIOOutput Update() {
        PhotonPipelineResult result = _camera.getLatestResult();
        _estimator.setFieldTags(FieldConstants.getFieldLayout());
        Optional<EstimatedRobotPose> currentPose = _estimator.update(result);
        
        if(!currentPose.isPresent())
            return _output;

        _output.hasTargets = true;
        
        _targets = currentPose.get().targetsUsed;
        findMinMax(_output);

        if (_output.maxAmbiguity < VisionConstants.maxAmbiguity) {
            _output.timestamp = currentPose.get().timestampSeconds;
            
            _output.robotPose = new Pose2d(
                currentPose.get().estimatedPose.getX(),
                currentPose.get().estimatedPose.getY(),
                Rotation2d.fromRadians(currentPose.get().estimatedPose.getRotation().getZ())
            );
        }

        return _output;
    }

    private void findMinMax(VisionIOOutput output) {
        output.closestTagDist = Double.MAX_VALUE;
        output.farthestTagDist = 0;
        output.maxAmbiguity = 0;

        for (PhotonTrackedTarget target : _targets) {
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            double ambiguity = target.getPoseAmbiguity();

            if (distance < output.closestTagDist){
                output.closestTagDist = distance;
                output.closestTagId = target.getFiducialId();
            }

            if (distance > output.farthestTagDist){
                output.farthestTagDist = distance;
                output.farthestTagId = target.getFiducialId();
            }

            if (ambiguity > output.maxAmbiguity){
                output.maxAmbiguity = ambiguity;
                output.maxAmbiguityTagId = target.getFiducialId();
            }
        }
    }
}