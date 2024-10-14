package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.NinjasLib.Swerve.SwerveIO;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
	private CommandPS5Controller _driverJoystick;
	private CommandPS5Controller _operatorJoystick;

	private boolean isSwerveLookAt = false;
	private boolean isSwerveBayblade = false;

	public RobotContainer() {
		RobotState.initPoseEstimator();

		AutoCommandBuilder.configureAutoBuilder();
		AutoCommandBuilder.registerCommands();

		Indexer.getInstance();
		Shooter.getInstance();
		ShooterAngle.getInstance();
		Constants.VisionConstants.getFieldLayout();

		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

		configureBindings();
	}

	private void configureBindings() {
		StateMachine.getInstance().setTriggerForSimulationTesting(_driverJoystick.R2());

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
		_driverJoystick.square().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE));
		_driverJoystick.triangle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT));
	}

	private void configureTestBindings() {
		_driverJoystick.triangle().whileTrue(Indexer.getInstance().runMotor(-1));
		_driverJoystick.cross().whileTrue(Indexer.getInstance().runMotor(1));

		_driverJoystick.povDown().whileTrue(ShooterAngle.getInstance().runMotor(-0.5));
		_driverJoystick.povUp().whileTrue(ShooterAngle.getInstance().runMotor(0.5));

		_driverJoystick.square().whileTrue(Shooter.getInstance().runMotor(1));
		_driverJoystick.circle().whileTrue(Shooter.getInstance().runMotor(1));
	}

	public void periodic() {
		SmartDashboard.putBoolean("Indexer Beam Breaker", RobotState.getNoteInIndexer());
		SmartDashboard.putNumber("Robot Velocity", RobotState.getRobotVelocity().getNorm());

//		ArrayList<Trajectory.State> positions = new ArrayList<>();
//		for (double t = 0; t < 5; t += 0.1) {
//			positions.add(new Trajectory.State());
//		}
//
//		Field2d shootingTraj = new Field3d();
//		shootingTraj.getObject("shootingTraj").setTrajectory(new Trajectory());

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

	public void teleopInit() {
//		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
//		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);
//
//		configureBindings();
	}

	public void testInit() {
//		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
//		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);
//
//		configureTestBindings();
	}
}
