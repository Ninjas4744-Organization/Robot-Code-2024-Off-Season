package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTag;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.DataClasses.VisionOutput;

public class Vision extends VisionIO {

	public void periodic() {
		for (int i = 0; i < _cameras.length; i++) {
			VisionOutput output = _cameras[i].Update();
			_outputs.put(_cameras[i].getName(), output);
			_estimationsData[i] = new VisionEstimation(output.robotPose, output.timestamp, output.hasTargets);
		}
	}

	/**
	 * @return an array of each camera's robot pose, the time when this pose was detected and if
	 * it
	 *     has targets
	 */
	public VisionEstimation[] getVisionEstimations() {
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

		for (String camera : _outputs.keySet()) {
			if (_outputs.get(camera).hasTargets) {
				hasTargets = true;
				break;
			}
		}

		return hasTargets;
	}
}
