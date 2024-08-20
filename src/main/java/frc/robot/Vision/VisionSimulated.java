package frc.robot.Vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.DataClasses.VisionOutput;
import frc.robot.RobotState;
import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSimulated extends VisionIO {
	private VisionSystemSim _visionSystemSim = new VisionSystemSim("main");
	private SimCameraProperties[] _cameraProperties;
	private PhotonCameraSim[] _simulatedCameras;

	public VisionSimulated() {
		super();

		_visionSystemSim.addAprilTags(VisionConstants.getFieldLayout());

		int camerasCount = _cameras.length;
		_simulatedCameras = new PhotonCameraSim[camerasCount];
		_cameraProperties = new SimCameraProperties[camerasCount];

		for(int i = 0; i < camerasCount; i++){
			_cameraProperties[i] = new SimCameraProperties();
			_cameraProperties[i].setCalibration(VisionConstants.Simulation.kResolutionWidth, VisionConstants.Simulation.kResolutionHeight, Rotation2d.fromDegrees(VisionConstants.Simulation.kFOV));
			_cameraProperties[i].setCalibError(VisionConstants.Simulation.kAverageError, VisionConstants.Simulation.kErrorStdDev);
			_cameraProperties[i].setFPS(VisionConstants.Simulation.kFPS);
			_cameraProperties[i].setAvgLatencyMs(VisionConstants.Simulation.kAverageLatency);
			_cameraProperties[i].setLatencyStdDevMs(VisionConstants.Simulation.kLatencyStdDev);

			_simulatedCameras[i] = new PhotonCameraSim(_cameras[i].getCamera(), _cameraProperties[i]);

			_visionSystemSim.addCamera(_simulatedCameras[i], VisionConstants.getCamerasPoses().get(_cameras[i].getName()));
		}
	}

	@Override
	public void periodic() {
		super.periodic();
		_visionSystemSim.update(RobotState.getRobotPose());
	}
}
