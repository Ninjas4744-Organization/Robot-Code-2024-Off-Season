package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Rotation;
import frc.robot.Swerve.SwerveIO;
import frc.robot.Vision.Vision;

public class RobotContainer {
	private CommandPS5Controller _driverJoystick;
	//	private Joystick _driverJoystick2;
	private CommandPS5Controller _operatorJoystick;

	public RobotContainer() {
		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
		//		_driverJoystick2 = new Joystick(1);

		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

		RobotState.initPoseEstimator();

		AutoCommandBuilder.configureAutoBuilder();
		AutoCommandBuilder.registerCommands();
		configureBindings();
	}

	private void configureBindings() {
//		new Trigger(RobotState::atSpeaker).onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_SHOOT));
//		new Trigger(RobotState::atAmp).onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_AMP_OUTAKE));
//		new Trigger(RobotState::atSource).onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_INTAKE));
//
//		new Trigger(() -> Elevator.getInstance().isHomed()
//						&& Rotation.getInstance().isHomed()
//						&& Climber.getInstance().isHomed())
//				.onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.IDLE));

		//		new Trigger(() -> {
		//					Pose2d _currentPose = RobotState.getRobotPose();
		//					return _currentPose.getX() < 3.5 && _currentPose.getY() > 6.5;
		//				})
		//				.onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_AMP_OUTAKE));

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

		_driverJoystick.triangle().onTrue(Commands.startEnd(
						() -> SwerveIO.getInstance().turnAnglePID(false),
						() -> SwerveIO.getInstance().turnAnglePID(true),
						SwerveIO.getInstance()));

		 _driverJoystick
				.L1()
				.onTrue(TeleopCommandBuilder.resetGyro(false));

		 _driverJoystick
				.L2()
				.onTrue(TeleopCommandBuilder.resetGyro(true));

		_driverJoystick
				.R1()
				.toggleOnTrue(Commands.startEnd(
						() -> SwerveIO.getInstance().setIsDriveAssist(true),
						() -> SwerveIO.getInstance().setIsDriveAssist(false)));

		_driverJoystick.R2().whileTrue(TeleopCommandBuilder.goToTag());
	}

	private void configureOperatorBindings() {
		_driverJoystick.cross().onTrue(StateMachine.getInstance().Act());
		_driverJoystick.triangle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_CLIMB));
		_driverJoystick.circle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.RESET));
	}

	public void periodic() {
		VisionEstimation[] estimations = Vision.getInstance().getVisionEstimations();

		for (VisionEstimation estimation : estimations) if (estimation != null) RobotState.updateRobotPose(estimation);
	}

	public void resetSubsystems() {
		RobotState.setRobotPose(new Pose2d());
		StateMachine.getInstance().changeRobotState(RobotStates.RESET);
		StateMachine.getInstance().changeRobotState(RobotStates.IDLE);
		TeleopCommandBuilder.resetGyro(false).schedule();
	}
}
