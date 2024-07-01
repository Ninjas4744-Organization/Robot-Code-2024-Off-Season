package frc.robot.Vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private VisionCamera[] _cameras;
    private EstimationData[] _estimationsData;
    private VisionOutput[] _outputs;

    /**
     * @param cameras Array of cameras being used
     */
    public Vision(VisionCamera[] cameras) {
        this._cameras = cameras;

        _estimationsData = new EstimationData[_cameras.length];
        _outputs = new VisionOutput[_cameras.length];
    }

    public void periodic() {
        for (int i = 0; i < _cameras.length; i++) {
            VisionOutput output = _cameras[i].Update();
            _outputs[i] = output;

            _estimationsData[i].pose = output.robotPose;
            _estimationsData[i].timestamp = output.timestamp;
            _estimationsData[i].hasTargets = output.hasTargets;
        }
    }

    /**
     * @return an array of each camera's robot pose, the time when this pose was detected and if it has targets
     */
    public EstimationData[] getEstimationsData(){
        return _estimationsData;
    }

    /**
     * @param camera - the camera to get info from
     * @return distance from the closest tag to this camera
     */
    public double getClosestTagDistance(int camera) {
        return _outputs[camera].closestTagDist;
    }

    /**
     * @param camera - the camera to get info from
     * @return id of closest tag to this camera
     */
    public double getClosestTagId(int camera) {
        return _outputs[camera].closestTagId;
    }

    /**
     * @param camera - the camera to get info from
     * @return distance from the farthest tag to this camera
     */
    public double getFarthestTagDistance(int camera) {
        return _outputs[camera].farthestTagDist;
    }

    /**
     * @param camera - the camera to get info from
     * @return id of farthest tag to this camera
     */
    public double getFarthestTagId(int camera) {
        return _outputs[camera].farthestTagId;
    }

    /**
     * @param camera - the camera to get info from
     * @return ambiguity of the most ambiguous tag from this camera
     */
    public double getMaxAmbiguity(int camera) {
        return _outputs[camera].maxAmbiguity;
    }

    /**
     * @param camera - the camera to get info from
     * @return id of most ambiguous tag from this camera
     */
    public double getMaxAmbiguityTagId(int camera) {
        return _outputs[camera].maxAmbiguityTagId;
    }
}
