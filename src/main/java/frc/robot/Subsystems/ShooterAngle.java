package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.StateMachineMotoredSubsystem;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;

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

	@Override
	protected void setFunctionMaps() {
		addFunctionToPeriodicMap(
				() -> {
					double dist = RobotState.getRobotPose()
							.getTranslation()
							.getDistance(VisionConstants.getTagPose(15).getTranslation());

					Rotation2d angle = Rotation2d.fromRadians(Math.atan2(
							ShooterAngleConstants.kTargetHeight - ShooterAngleConstants.kShooterHeight, dist));
					double angleClamped =
							Math.min(Math.max(angle.getDegrees() - ShooterAngleConstants.kShooterStartAngle, 0), 120);

					controller().setPosition(angleClamped);
				},
				RobotState.RobotStates.PREPARE_SHOOT);
	}
}
