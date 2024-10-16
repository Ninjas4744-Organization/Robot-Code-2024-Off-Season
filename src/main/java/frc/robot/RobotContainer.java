package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.NinjasLib.DataClasses.VisionEstimation;
import frc.robot.NinjasLib.Swerve.SwerveIO;
import frc.robot.NinjasLib.Vision.VisionIO;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;

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

//				_driverJoystick.square().onTrue(Commands.runOnce(() -> isSwerveBayblade = !isSwerveBayblade));
		_driverJoystick.R1().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));
		_driverJoystick.R2().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT));

		_driverJoystick.L1().onTrue(TeleopCommandBuilder.resetGyro(false));
		_driverJoystick.L2().onTrue(TeleopCommandBuilder.resetGyro(true));
	}

	private void configureOperatorBindings() {
		_driverJoystick.cross().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.INTAKE));
		_driverJoystick.triangle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE));
		_driverJoystick.square().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_AMP_PREPARE));
		_driverJoystick.circle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.CLOSE));
		_driverJoystick.R2().onTrue(Commands.runOnce(() -> {
			StateMachine.getInstance().changeRobotState(RobotStates.SHOOT);
			StateMachine.getInstance().changeRobotState(RobotStates.OUTTAKE);
		}, StateMachine.getInstance()));
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

		VisionEstimation[] estimations = VisionIO.getInstance().getVisionEstimations();
		RobotState.updateRobotPose(estimations);
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
