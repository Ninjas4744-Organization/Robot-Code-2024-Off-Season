package frc.robot;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Rotation;
import frc.robot.Swerve.Swerve;

public class RobotContainer {
	private CommandPS5Controller _driverJoystick;
	private Joystick _driverJoystick2;
	private CANSparkMax _test; // Remove
	private CommandPS5Controller _operatorJoystick;

	public RobotContainer() {
		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
		_driverJoystick2 = new Joystick(1);

		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

		if (Robot.isReal()) RobotState.initPoseEstimator();

		AutoCommandBuilder.configureAutoBuilder();
		AutoCommandBuilder.registerCommands();
		RobotState.setRobotState(RobotStates.IDLE);
		configureBindings();
	}

	private void configureBindings() {
		// new Trigger(() -> Vision.getInstance().atAmp())
		// .onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_AMP_OUTAKE));
		// new Trigger(() -> Vision.getInstance().atSource())
		// .onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_INTAKE));
		// new Trigger(() -> Vision.getInstance().atSpeaker())
		// .onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_SHOOT));

		new Trigger(() -> Elevator.getInstance().isHomed()
						&& Rotation.getInstance().isHomed()
						&& Climber.getInstance().isHomed())
				.onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.IDLE));

		configureDriverBindings();
		configureOperatorBindings();
	}

	private void configureDriverBindings() {
		Swerve.getInstance()
				.setDefaultCommand(TeleopCommandBuilder.swerveDrive(
						() -> new Translation2d(_driverJoystick.getLeftX(), _driverJoystick.getLeftY()),
						() -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY()),
						true));

		_driverJoystick
				.circle()
				.toggleOnTrue(
						Commands.startEnd(() -> Swerve.getInstance().setBaybladeMode(true), () -> Swerve.getInstance()
								.setBaybladeMode(false)));

		_driverJoystick
				.L1()
				.onTrue(Commands.parallel(
						TeleopCommandBuilder.resetGyro(false),
						Commands.runOnce(() -> Swerve.getInstance().resetModulesToAbsolute(), Swerve.getInstance())));

		_driverJoystick
				.L2()
				.onTrue(Commands.parallel(
						TeleopCommandBuilder.resetGyro(true),
						Commands.runOnce(() -> Swerve.getInstance().resetModulesToAbsolute(), Swerve.getInstance())));

		_driverJoystick
				.R1()
				.toggleOnTrue(
						Commands.startEnd(() -> Swerve.getInstance().setIsDriveAssist(true), () -> Swerve.getInstance()
								.setIsDriveAssist(false)));

		// _driverJoystick.R2().whileTrue(TeleopCommandBuilder.goToTag());
	}

	private void configureOperatorBindings() {
		_operatorJoystick.cross().onTrue(StateMachine.getInstance().Act());
		_operatorJoystick.triangle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_CLIMB));
		_operatorJoystick.circle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.RESET));
	}

	public void periodic() {
		// VisionEstimation[] estimations = Vision.getInstance().getVisionEstimations();
		//
		// for (VisionEstimation estimation : estimations)
		// if (estimation != null) RobotState.updateRobotPose(estimation);
	}

	public void resetSubsystems() {
		StateMachine.getInstance().changeRobotState(RobotStates.RESET);
		TeleopCommandBuilder.resetGyro(false).schedule();
	}
}
