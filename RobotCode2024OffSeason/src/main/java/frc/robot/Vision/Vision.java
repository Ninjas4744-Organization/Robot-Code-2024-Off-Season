package frc.robot.Vision;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    private VisionCamera[] _cameras;
    private VisionEstimation[] _estimationsData;
    private HashMap<String, VisionOutput> _outputs;

    /**
     * @param cameras Array of the names of the cameras being used
     */
    public Vision(String[] cameras) {
        _cameras = new VisionCamera[cameras.length];

        HashMap<String, Transform3d> cameraPoses = Constants.VisionConstants.getCameraPoses();
        for (int i = 0; i < cameras.length; i++) {
            _cameras[i] = new VisionCamera(cameras[i], cameraPoses.get(cameras[i]));
        }

        _estimationsData = new VisionEstimation[_cameras.length];
        _outputs = new HashMap<String, VisionOutput>();
    }

    public void periodic() {
        for (int i = 0; i < _cameras.length; i++) {
            VisionOutput output = _cameras[i].Update();
            _outputs.put(_cameras[i].getName(), output);

            _estimationsData[i].pose = output.robotPose;
            _estimationsData[i].timestamp = output.timestamp;
            _estimationsData[i].hasTargets = output.hasTargets;
        }
    }

    /**
     * @return an array of each camera's robot pose, the time when this pose was detected and if it has targets
     */
    public VisionEstimation[] getEstimationsData(){
        return _estimationsData;
    }

    /**
     * @param camera - the name of the name of the camera to get info from
     * @return distance from the closest tag to this camera
     */
    public double getClosestTagDistance(String camera) {
        return _outputs.get(camera).closestTagDist;
    }

    /**
     * @param camera - the name of the camera to get info from
     * @return closest tag to this camera
     */
    public AprilTag getClosestTag(String camera) {
        return _outputs.get(camera).closestTag;
    }

    /**
     * @param camera - the name of the camera to get info from
     * @return distance from the farthest tag to this camera
     */
    public double getFarthestTagDistance(String camera) {
        return _outputs.get(camera).farthestTagDist;
    }

    /**
     * @param camera - the name of the camera to get info from
     * @return farthest tag to this camera
     */
    public AprilTag getFarthestTag(String camera) {
        return _outputs.get(camera).farthestTag;
    }

    /**
     * @param camera - the name of the camera to get info from
     * @return ambiguity of the most ambiguous tag from this camera
     */
    public double getMostAmbiguousTagAmbiguity(String camera) {
        return _outputs.get(camera).maxAmbiguity;
    }

    /**
     * @param camera - the name of the camera to get info from
     * @return most ambiguous tag from this camera
     */
    public AprilTag getMostAmbiguousTag(String camera) {
        return _outputs.get(camera).maxAmbiguityTag;
    }

    /**
     * @param camera - the name of the camera to get info from
     * @return if this camera has targets
     */
    public boolean hasTargets(String camera) {
        return _outputs.get(camera).hasTargets;
    }

    /**
     * @return if any of the cameras have targets
     */
    public boolean hasTargets() {
        boolean hasTargets = false;

        for(String camera : _outputs.keySet()) {
            if(_outputs.get(camera).hasTargets) {
                hasTargets = true;
                break;
            }
        }

        return hasTargets;
    }
}
