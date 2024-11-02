package frc.robot.NinjasLib.Vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NinjasLib.DataClasses.VisionConstants;
import frc.robot.NinjasLib.DataClasses.VisionEstimation;
import frc.robot.NinjasLib.DataClasses.VisionOutput;
import frc.robot.NinjasLib.RobotStateIO;

import java.util.Arrays;
import java.util.HashMap;

public abstract class VisionIO extends SubsystemBase {
	private static VisionIO _instance;
	protected HashMap<String, VisionOutput> _outputs;
	protected VisionEstimation[] _visionEstimations;
	protected VisionCamera[] _cameras;

	public static VisionIO getInstance() {
		if (_instance == null)
			throw new RuntimeException("VisionIO constants not given. Initialize VisionIO by setConstants(VisionConstants) first.");
		return _instance;
	}

	public static void setConstants(VisionConstants constants) {
		if (_instance == null) {
			if (RobotStateIO.getInstance().isSimulated()) _instance = new VisionSimulated(constants);
			else _instance = new Vision(constants);
		}
	}

	protected VisionIO(VisionConstants constants) {
		String[] camerasNames = constants.cameras.keySet().toArray(new String[0]);

		_cameras = new VisionCamera[camerasNames.length];
		for (int i = 0; i < constants.cameras.size(); i++)
			_cameras[i] = new VisionCamera(camerasNames[i], constants.cameras.get(camerasNames[i]), constants);

		_visionEstimations = new VisionEstimation[camerasNames.length];
		_outputs = new HashMap<>();
		for (String name : camerasNames) {
			VisionOutput output = new VisionOutput();
			_outputs.put(name, output);

			_visionEstimations[Arrays.asList(camerasNames).indexOf(name)] = new VisionEstimation(output.robotPose, output.timestamp, output.hasTargets, output.closestTag.pose.toPose2d());
		}
	}

	@Override
	public void periodic() {
		for (int i = 0; i < _cameras.length; i++) {
			VisionOutput output = _cameras[i].Update();
			_outputs.put(_cameras[i].getName(), output);
			_visionEstimations[i] = new VisionEstimation(output.robotPose, output.timestamp, output.hasTargets, output.closestTag.pose.toPose2d());
		}
	}

	/**
	 * @return an array of each camera's robot pose, the time when this pose was detected and if
	 * it has targets
	 */
	public VisionEstimation[] getVisionEstimations() {
		return _visionEstimations;
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
		for (String camera : _outputs.keySet()) {
			if (hasTargets(camera)) return true;
		}
		return false;
	}

	public void ignoreTag(int id) {
		for (VisionCamera camera : _cameras) camera.ignoreTag(id);
	}
}
