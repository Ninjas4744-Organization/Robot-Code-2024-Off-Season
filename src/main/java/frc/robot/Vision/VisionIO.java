package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.DataClasses.VisionOutput;
import frc.robot.RobotState;
import java.util.HashMap;

public abstract class VisionIO extends SubsystemBase {
	private static VisionIO _instance;
	protected HashMap<String, VisionOutput> _outputs;
	protected VisionEstimation[] _estimationsData;
	protected VisionCamera[] _cameras;

	public static VisionIO getInstance() {
		if (_instance == null) {
			if (RobotState.isSimulated()) _instance = new VisionSimulated();
			else _instance = new Vision();
		}
		return _instance;
	}

	public VisionIO() {
		HashMap<String, Transform3d> camerasPoses = Constants.VisionConstants.getCamerasPoses();
		String[] camerasNames = camerasPoses.keySet().toArray(new String[0]);

		_cameras = new VisionCamera[camerasNames.length];
		for (int i = 0; i < camerasPoses.size(); i++)
			_cameras[i] = new VisionCamera(camerasNames[i], camerasPoses.get(camerasNames[i]));

		_estimationsData = new VisionEstimation[camerasNames.length];
		_outputs = new HashMap<String, VisionOutput>();
		for (String name : camerasNames) {
			VisionOutput output = new VisionOutput();
			_outputs.put(name, output);
		}
	}

	@Override
	public void periodic() {
		for (int i = 0; i < _cameras.length; i++) {
			VisionOutput output = _cameras[i].Update();
			_outputs.put(_cameras[i].getName(), output);
			_estimationsData[i] = new VisionEstimation(output.robotPose, output.timestamp, output.hasTargets);
		}
	}

	/**
	 * @return an array of each camera's robot pose, the time when this pose was detected and if
	 * it has targets
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
