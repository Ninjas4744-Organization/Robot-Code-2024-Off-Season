package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.RobotState;

public class ShooterAngle extends StateMachineMotoredSubsystem {
	private static ShooterAngle _instance;
	private DigitalInput _limit;

	public ShooterAngle(boolean disabled) {
		super(disabled);

		_limit = new DigitalInput(ShooterAngleConstants.kLimitSwitchId);
	}

	public static void disable() {
		if (_instance == null)
			_instance = new ShooterAngle(true);
	}

	public static ShooterAngle getInstance() {
		if (_instance == null) _instance = new ShooterAngle(false);

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
	public void resetSubsystem() {
		controller().stop();
	}

	@Override
	public boolean isResetted() {
//		return controller().isHomed();
		return true;
	}

	private Rotation2d calculateAngle(Pose3d target) {
		double dist = RobotState.getRobotPose()
				.getTranslation()
				.plus(ShooterAngleConstants.kShooterPose.toTranslation2d())
				.getDistance(target.toPose2d().getTranslation());

		Rotation2d angle =
				Rotation2d.fromRadians(Math.atan2(target.getZ() - ShooterAngleConstants.kShooterPose.getZ(), dist));

		angle = angle.rotateBy(ShooterAngleConstants.getTrendAngleFixer(dist));

		double angleClamped = Math.min(Math.max(angle.getDegrees(), 40), 80);

		return Rotation2d.fromDegrees(angleClamped);
	}

	@Override
	protected void setFunctionMaps() {
//		addFunctionToPeriodicMap(
//				() -> controller()
//						.setPosition(calculateAngle(new Pose3d(
//										VisionConstants.getAmpTag().pose.getX()
//												+ ShooterAngleConstants.kAmpOffset.getX(),
//										VisionConstants.getAmpTag().pose.getY()
//												+ ShooterAngleConstants.kAmpOffset.getY(),
//										VisionConstants.getAmpTag().pose.getZ()
//												+ ShooterAngleConstants.kAmpOffset.getZ(),
//										new Rotation3d()))
//								.getDegrees()),
//				RobotStates.SHOOT_AMP_PREPARE);
//
//		addFunctionToPeriodicMap(
//				() -> controller()
//						.setPosition(calculateAngle(new Pose3d(
//										VisionConstants.getSpeakerTag().pose.getX()
//												+ ShooterAngleConstants.kSpeakerOffset.getX(),
//										VisionConstants.getSpeakerTag().pose.getY()
//												+ ShooterAngleConstants.kSpeakerOffset.getY(),
//										VisionConstants.getSpeakerTag().pose.getZ()
//												+ ShooterAngleConstants.kSpeakerOffset.getZ(),
//										new Rotation3d()))
//								.getDegrees()),
//				RobotStates.SHOOT_SPEAKER_PREPARE);

//		addFunctionToPeriodicMap(
//				() -> controller().setPosition(65),
//				RobotStates.SHOOT_SPEAKER_PREPARE);
	}

	@Override
	public boolean atGoal() {
		return true;
	}

	@Override
	public void periodic() {
		super.periodic();

		SmartDashboard.putBoolean("Shooter Angle Limit", _limit.get());
		if (_limit.get()) {
			_controller.resetEncoder();
			if (_controller.getOutput() < 0)
				_controller.stop();
		}
	}
}
