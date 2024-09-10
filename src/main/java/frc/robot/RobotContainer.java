package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Shooter;
import frc.robot.Swerve.SwerveDemand;
import frc.robot.Swerve.SwerveIO;
import frc.robot.Vision.VisionIO;

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

		Shooter.getInstance();

		configureBindings();
	}

	private void configureBindings() {
		//		new Trigger(RobotState::atSpeaker).onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_SHOOT));
		//		new
		// Trigger(RobotState::atAmp).onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.PREPARE_AMP_OUTAKE));
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
					() -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY())));

		_driverJoystick.circle().toggleOnTrue(Commands.runOnce(
			() -> {
				if (SwerveIO.getInstance().getState() == SwerveDemand.SwerveState.BAYBLADE)
					SwerveIO.getInstance().setState(SwerveIO.getInstance().getPreviousState());
				else
					SwerveIO.getInstance().setState(SwerveDemand.SwerveState.BAYBLADE);
			}));

		_driverJoystick.triangle().onTrue(Commands.runOnce(
			() -> {
				if (SwerveIO.getInstance().getState() == SwerveDemand.SwerveState.LOOK_AT_ANGLE)
					SwerveIO.getInstance().setState(SwerveIO.getInstance().getPreviousState());
				else
					SwerveIO.getInstance().setState(SwerveDemand.SwerveState.LOOK_AT_ANGLE);
			}));

		_driverJoystick.povUp().toggleOnTrue(Commands.runOnce(
			() -> {
				if (SwerveIO.getInstance().getState() == SwerveDemand.SwerveState.LOOK_AT_TARGET)
					SwerveIO.getInstance().setState(SwerveIO.getInstance().getPreviousState());
				else
					SwerveIO.getInstance().setState(SwerveDemand.SwerveState.LOOK_AT_TARGET);
			}));

		_driverJoystick.L1().onTrue(TeleopCommandBuilder.resetGyro(false));
		_driverJoystick.L2().onTrue(TeleopCommandBuilder.resetGyro(true));

		_driverJoystick.square().onTrue(Commands.runOnce(() -> {
			SwerveIO.getInstance().setState(SwerveDemand.SwerveState.LOCKED_AXIS);
//			SwerveIO.getInstance().updateDemand(Rotation2d.fromDegrees(-60), 6.25);
			SwerveIO.getInstance().updateDemand(Rotation2d.fromDegrees(0), 7.6);
		}));

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
