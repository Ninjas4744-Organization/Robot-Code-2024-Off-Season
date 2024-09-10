package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.SwerveIO;
import frc.robot.Vision.VisionIO;

public class RobotContainer {
	private CommandPS5Controller _driverJoystick;
	private CommandPS5Controller _operatorJoystick;

	public RobotContainer() {
		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);

		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

		RobotState.initPoseEstimator();

		AutoCommandBuilder.configureAutoBuilder();
		AutoCommandBuilder.registerCommands();

		Shooter.getInstance();

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
						true));

		_driverJoystick
				.circle()
				.toggleOnTrue(Commands.startEnd(
						() -> SwerveIO.getInstance().setBaybladeMode(true),
						() -> SwerveIO.getInstance().setBaybladeMode(false)));

		_driverJoystick
				.triangle()
				.onTrue(Commands.runOnce(
						() -> SwerveIO.getInstance()
								.setAnglePID(!SwerveIO.getInstance().isAnglePID()),
						SwerveIO.getInstance()));

		_driverJoystick.L1().onTrue(TeleopCommandBuilder.resetGyro(false));

		_driverJoystick.L2().onTrue(TeleopCommandBuilder.resetGyro(true));

		_driverJoystick
				.R1()
				.toggleOnTrue(Commands.startEnd(
						() -> SwerveIO.getInstance().setDriveAssist(true),
						() -> SwerveIO.getInstance().setDriveAssist(false)));
	}

	private void configureOperatorBindings() {
		//		_driverJoystick.cross().onTrue(StateMachine.getInstance().Act());
		//		_driverJoystick.triangle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_CLIMB));
		//		_driverJoystick.circle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.RESET));
	}

	public void periodic() {
		VisionEstimation[] estimations = VisionIO.getInstance().getVisionEstimations();

		for (VisionEstimation estimation : estimations) if (estimation != null) RobotState.updateRobotPose(estimation);
	}

	public void resetSubsystems() {
		RobotState.setRobotPose(new Pose2d());
		StateMachine.getInstance().changeRobotState(RobotStates.RESET);
		StateMachine.getInstance().changeRobotState(RobotStates.IDLE);
		TeleopCommandBuilder.resetGyro(false).schedule();
	}
}
