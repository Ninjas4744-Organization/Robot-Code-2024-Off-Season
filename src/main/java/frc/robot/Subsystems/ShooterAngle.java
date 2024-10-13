package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ballistics;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasSparkMaxController;
import frc.robot.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;

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

	private Rotation2d calculateAngle(Pose3d target, double shootSpeed) {
		double dist = RobotState.getRobotPose()
				.getTranslation()
				.plus(ShooterAngleConstants.kShooterPose.toTranslation2d())
				.getDistance(target.toPose2d().getTranslation());

		SmartDashboard.putNumber("Distance", dist);
		SmartDashboard.putNumber("Trigo Angle", ShooterAngleConstants.getEnterAngle(new Translation3d(
			target.getX() - (RobotState.getRobotPose().getX() + ShooterAngleConstants.kShooterPose.getX()),
			target.getY() - (RobotState.getRobotPose().getY() + ShooterAngleConstants.kShooterPose.getY()),
			target.getZ() - ShooterAngleConstants.kShooterPose.getZ()
		)));
		NetworkTableInstance.getDefault().getTable("Shooter Target").getEntry("Shooter Target").setDoubleArray(new double[]{target.getX(), target.getY(), target.getZ()});
		NetworkTableInstance.getDefault().getTable("Shooter Target").getEntry("Shooter Target XY").setDoubleArray(new double[]{target.getX(), target.getY(), ShooterAngleConstants.kShooterPose.getZ()});
		SmartDashboard.putNumber("Note Shooting Speed", ShooterAngleConstants.shooterSpeedToNoteSpeed(shootSpeed));

		double angle = Ballistics.calculateShootingAngle(
			ShooterAngleConstants.kShooterPose.getZ(),
			dist,
			target.getZ(),
			ShooterAngleConstants.shooterSpeedToNoteSpeed(shootSpeed),
			ShooterAngleConstants.getEnterAngle(new Translation3d(
				target.getX() - (RobotState.getRobotPose().getX() + ShooterAngleConstants.kShooterPose.getX()),
				target.getY() - (RobotState.getRobotPose().getY() + ShooterAngleConstants.kShooterPose.getY()),
				target.getZ() - ShooterAngleConstants.kShooterPose.getZ()
			)));

		double angleClamped = Math.min(Math.max(angle, 40), 80);
		return Rotation2d.fromDegrees(angleClamped);
	}

	@Override
	protected void setFunctionMaps() {
//		addFunctionToPeriodicMap(
//				() -> controller()
//						.setPosition(calculateAngle(new Pose3d(
//										VisionConstants.getAmpTag().pose.getX()
//												+ ShooterAngleConstants.kAmpOffset.getX() * (RobotState.getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
//										VisionConstants.getAmpTag().pose.getY()
//												+ ShooterAngleConstants.kAmpOffset.getY() * (RobotState.getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
//										VisionConstants.getAmpTag().pose.getZ()
//												+ ShooterAngleConstants.kAmpOffset.getZ(),
//										new Rotation3d()), ShooterConstants.States.kAmp)
//								.getDegrees()),
//				RobotStates.SHOOT_AMP_PREPARE);
//
//		addFunctionToPeriodicMap(
//				() -> controller()
//						.setPosition(calculateAngle(new Pose3d(
//										VisionConstants.getSpeakerTag().pose.getX()
//												+ ShooterAngleConstants.kSpeakerOffset.getX() * (RobotState.getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
//										VisionConstants.getSpeakerTag().pose.getY()
//												+ ShooterAngleConstants.kSpeakerOffset.getY() * (RobotState.getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
//										VisionConstants.getSpeakerTag().pose.getZ()
//												+ ShooterAngleConstants.kSpeakerOffset.getZ(),
//										new Rotation3d()), ShooterConstants.States.kSpeaker)
//								.getDegrees()),
//				RobotStates.SHOOT_SPEAKER_PREPARE);

		addFunctionToOnChangeMap(
			() -> controller().setPosition(40),
			RobotStates.SHOOT_SPEAKER_PREPARE);
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
