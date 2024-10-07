package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;

public class ShooterAngle extends StateMachineMotoredSubsystem {
	private static ShooterAngle _instance;

	public static ShooterAngle getInstance() {
		if (_instance == null) _instance = new ShooterAngle();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ShooterAngleConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(ShooterAngleConstants.kSimulatedControllerConstants);
	}

	private Rotation2d calculateAngle(Pose3d target){
		double dist = RobotState.getRobotPose()
			.getTranslation().plus(ShooterAngleConstants.kShooterPose.toTranslation2d())
			.getDistance(target.toPose2d().getTranslation());

		Rotation2d angle = Rotation2d.fromRadians(Math.atan2(
			target.getZ() - ShooterAngleConstants.kShooterPose.getZ(), dist));

		angle = angle.rotateBy(ShooterAngleConstants.getTrendAngleFixer(dist));

		double angleClamped =
			Math.min(Math.max(angle.getDegrees(), 30), 80);

		return Rotation2d.fromDegrees(angleClamped);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToPeriodicMap(
			() -> controller().setPosition(calculateAngle(
					new Pose3d(VisionConstants.getAmpTag().pose.getX() + ShooterAngleConstants.kAmpOffset.getX(),
							VisionConstants.getAmpTag().pose.getY() + ShooterAngleConstants.kAmpOffset.getY(),
							VisionConstants.getAmpTag().pose.getZ() + ShooterAngleConstants.kAmpOffset.getZ(),
							new Rotation3d())).getDegrees()),
				RobotStates.SHOOT_AMP_PREPARE);

		addFunctionToPeriodicMap(
				() -> controller().setPosition(calculateAngle(
						new Pose3d(VisionConstants.getSpeakerTag().pose.getX() + ShooterAngleConstants.kSpeakerOffset.getX(),
								VisionConstants.getSpeakerTag().pose.getY() + ShooterAngleConstants.kSpeakerOffset.getY(),
								VisionConstants.getSpeakerTag().pose.getZ() + ShooterAngleConstants.kSpeakerOffset.getZ(),
								new Rotation3d())).getDegrees()),
			RobotStates.SHOOT_SPEAKER_PREPARE);
	}
}
