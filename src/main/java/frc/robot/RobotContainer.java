package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.NinjasLib.Swerve.SwerveIO;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;

public class RobotContainer {
	private final CommandPS5Controller _driverJoystick;
	private final CommandPS5Controller _operatorJoystick;

	private boolean isSwerveLookAt = false;
	private boolean isSwerveBayblade = false;

	public RobotContainer() {
		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);

		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

		RobotState.initPoseEstimator();

		AutoCommandBuilder.configureAutoBuilder();
		AutoCommandBuilder.registerCommands();

		Climber.disable();
		Indexer.getInstance();
		Shooter.getInstance();
		ShooterAngle.getInstance();
		Constants.VisionConstants.getFieldLayout();

		StateMachine.getInstance().setTriggerForSimulationTesting(_driverJoystick.R2());

		configureBindings();
	}

	private void configureBindings() {
		configureDriverBindings();
		configureOperatorBindings();
	}

	private void configureDriverBindings() {
		SwerveIO.getInstance()
				.setDefaultCommand(TeleopCommandBuilder.swerveDrive(
						() -> new Translation2d(_driverJoystick.getLeftX(), _driverJoystick.getLeftY()),
						() -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY()),
						() -> isSwerveLookAt,
						() -> isSwerveBayblade));

//		_driverJoystick.square().onTrue(Commands.runOnce(() -> isSwerveBayblade = !isSwerveBayblade));
//		_driverJoystick.triangle().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));
//
//		_driverJoystick.L1().onTrue(TeleopCommandBuilder.resetGyro(false));
		_driverJoystick.L2().onTrue(TeleopCommandBuilder.resetGyro(true));
//
//		_driverJoystick.povLeft().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.DRIVE_TO_AMP));
//		_driverJoystick.povUp().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.DRIVE_TO_SOURCE));
//		_driverJoystick.povDown().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE));
	}

	private void configureOperatorBindings() {
		_driverJoystick.cross().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.INTAKE));
//		_driverJoystick.cross().toggleOnTrue(Indexer.getInstance().runMotor(0.5));
//		_driverJoystick.circle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.CLOSE));
		_driverJoystick.circle().toggleOnTrue(Indexer.getInstance().runMotor(Constants.IndexerConstants.States.kRoll));
		_driverJoystick.square().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE));
		_driverJoystick.triangle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT));
		_driverJoystick.povDown().whileTrue(ShooterAngle.getInstance().runMotor(-0.5));
		_driverJoystick.povUp().whileTrue(ShooterAngle.getInstance().runMotor(0.5));
		_driverJoystick.povLeft().onTrue(Commands.runOnce(() -> ShooterAngle.getInstance().resetLikeLimit()));
	}

	public void periodic() {
		SmartDashboard.putBoolean("Indexer Beam Breaker", RobotState.getNoteInIndexer());
		SmartDashboard.putNumber("Robot Velocity", RobotState.getRobotVelocity().getNorm());
//		VisionEstimation[] estimations = VisionIO.getInstance().getVisionEstimations();
//
//		for (VisionEstimation estimation : estimations) if (estimation != null) RobotState.updateRobotPose(estimation);
	}

	public void resetSubsystems() {
		RobotState.setRobotPose(new Pose2d());

		RobotState.setRobotState(RobotStates.RESET);
		Shooter.getInstance().resetSubsystem();
		Indexer.getInstance().resetSubsystem();
		ShooterAngle.getInstance().resetSubsystem();
		Climber.getInstance().resetSubsystem();

		TeleopCommandBuilder.resetGyro(false).schedule();
	}
}
