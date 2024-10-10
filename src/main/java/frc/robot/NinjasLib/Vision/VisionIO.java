package frc.robot.NinjasLib.Vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.NinjasLib.DataClasses.VisionEstimation;
import frc.robot.NinjasLib.DataClasses.VisionOutput;
import frc.robot.RobotState;

import java.util.HashMap;

public abstract class VisionIO extends SubsystemBase {
	private static VisionIO _instance;
	protected HashMap<String, VisionOutput> _outputs;
	protected VisionEstimation[] _estimationsData;
	protected VisionCamera[] _cameras;

	public static VisionIO getInstance() {
		System.out.println("Vision");

		if (_instance == null) {
			if (RobotState.isSimulated()) _instance = new VisionSimulated();
			else _instance = new Vision();
		}
		return _instance;
	}

	public VisionIO() {
		String[] camerasNames = VisionConstants.kCameras.keySet().toArray(new String[0]);

		_cameras = new VisionCamera[camerasNames.length];
		for (int i = 0; i < VisionConstants.kCameras.size(); i++)
			_cameras[i] = new VisionCamera(camerasNames[i], VisionConstants.kCameras.get(camerasNames[i]));

		_estimationsData = new VisionEstimation[camerasNames.length];
		_outputs = new HashMap<>();
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
		for (String camera : _outputs.keySet()) {
			if (hasTargets(camera)) return true;
		}
		return false;
	}

	/**
	 * Get the camera that its looking direction is closest to the given direction,
	 * for example if given direction (1, 0) and the robot is looking straight(gyro=0) it will return front camera
	 * @param dir The direction to find the closest camera to, field relative
	 * @return The name of the camera with the closest direction
	 */
	public String getCameraByDirection(Translation2d dir) {
		Rotation2d dirAngle = dir.getAngle().rotateBy(RobotState.getGyroYaw().unaryMinus());

		String closestCamera = "";
		Rotation2d closestAngleDiff = Rotation2d.fromDegrees(Double.MAX_VALUE);

		String[] camerasNames = VisionConstants.kCameras.keySet().toArray(new String[0]);

		for (String camera : camerasNames) {
			Rotation2d cameraAngle = Rotation2d.fromRadians(
					VisionConstants.kCameras.get(camera).getRotation().getZ());

			if (dirAngle.minus(cameraAngle).getDegrees() < closestAngleDiff.getDegrees()) {
				closestAngleDiff = dirAngle.minus(cameraAngle);
				closestCamera = camera;
			}
		}

		return closestCamera;
	}

	public void ignoreTag(int id) {
		for (VisionCamera camera : _cameras) camera.ignoreTag(id);
	}
}
