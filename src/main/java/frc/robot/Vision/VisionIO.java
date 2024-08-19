// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.DataClasses.VisionOutput;
import frc.robot.Robot;
import java.util.HashMap;

public abstract class VisionIO extends SubsystemBase {

	private static VisionIO _instance;
	protected HashMap<String, VisionOutput> _outputs;
	protected VisionEstimation[] _estimationsData;
	protected VisionCamera[] _cameras;

	public static VisionIO getInstance() {

		if (_instance == null) {
			if (Robot.isReal()) _instance = new Vision();
			else _instance = new SimVision();
		}
		return _instance;
	}
	/** Creates a new VisionIO. */
	VisionIO() {
		System.out.println("CREATED");
		HashMap<String, Transform3d> camerasPoses = Constants.VisionConstants.getCamerasPoses();
		String[] camerasNames = camerasPoses.keySet().toArray(new String[0]);

		_cameras = new VisionCamera[camerasNames.length];

		for (int i = 0; i < camerasPoses.size(); i++) {
			_cameras[i] = new VisionCamera(camerasNames[i], camerasPoses.get(camerasNames[i]));
		}

		_estimationsData = new VisionEstimation[_cameras.length];
		_outputs = new HashMap<String, VisionOutput>();
		for (int i = 0; i < _cameras.length; i++) {
			VisionOutput output = new VisionOutput();
			_outputs.put(_cameras[i].getName(), output);
		}
	}

	public VisionEstimation[] getVisionEstimations() {
		return _estimationsData;
	}
}
