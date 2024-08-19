package frc.robot.Vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.DataClasses.VisionOutput;
import frc.robot.RobotState;
import java.util.HashMap;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class SimVision extends VisionIO {

	// A vision system sim labelled as "main" in NetworkTables
	VisionSystemSim visionSim = new VisionSystemSim("main");

	// The simulated camera properties
	SimCameraProperties cameraProp = new SimCameraProperties();

	SimVision() {
		visionSim.addAprilTags(Constants.VisionConstants.getFieldLayout());

		// A 640 x 480 camera with a 100 degree diagonal FOV.
		cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
		// Approximate detection noise with average and standard deviation error in pixels.
		cameraProp.setCalibError(0.25, 0.08);
		// Set the camera image capture framerate (Note: this is limited by robot loop rate).
		cameraProp.setFPS(20);
		// The average and standard deviation in milliseconds of image data latency.
		cameraProp.setAvgLatencyMs(35);
		cameraProp.setLatencyStdDevMs(5);
		// The simulation of this camera. Its values used in real robot code will be updated.
		PhotonCameraSim cameraSim = new PhotonCameraSim(_cameras[0].getCamera(), cameraProp);
		HashMap<String, Transform3d> camerasPoses = Constants.VisionConstants.getCamerasPoses();
		String[] camerasNames = camerasPoses.keySet().toArray(new String[0]);

		Transform3d robotToCamera = camerasPoses.get(camerasNames[0]);

		// Add this camera to the vision system simulation with the given robot-to-camera transform.
		visionSim.addCamera(cameraSim, robotToCamera);
	}

	@Override
	public void periodic() {

		VisionOutput output = _cameras[0].Update();
		_outputs.put(_cameras[0].getName(), output);
		_estimationsData[0] = new VisionEstimation(output.robotPose, output.timestamp, output.hasTargets);
		SmartDashboard.putNumber("Tag Estimation", output.closestTag.ID);
		visionSim.update(RobotState.getRobotPose());
	}
}
