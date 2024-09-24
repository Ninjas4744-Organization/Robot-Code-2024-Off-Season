package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.ShooterAngle;
import frc.robot.Swerve.SwerveDemand;
import frc.robot.Swerve.SwerveIO;
import frc.robot.Vision.VisionIO;

public class RobotContainer {
	private final CommandPS5Controller _driverJoystick;
	private final CommandPS5Controller _operatorJoystick;

	public RobotContainer() {
		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);

		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

		RobotState.initPoseEstimator();

		AutoCommandBuilder.configureAutoBuilder();
		AutoCommandBuilder.registerCommands();

		ShooterAngle.getInstance();

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
						() -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY())));

		_driverJoystick.square().onTrue(Commands.runOnce(() -> {
			if (SwerveIO.getInstance().getState() == SwerveDemand.SwerveState.BAYBLADE)
				SwerveIO.getInstance().setState(SwerveIO.getInstance().getPreviousState());
			else SwerveIO.getInstance().setState(SwerveDemand.SwerveState.BAYBLADE);
		}));

		_driverJoystick.triangle().onTrue(Commands.runOnce(() ->
			SwerveIO.getInstance().setState(SwerveDemand.SwerveState.LOOK_AT_ANGLE), SwerveIO.getInstance()));

		_driverJoystick.L1().onTrue(TeleopCommandBuilder.resetGyro(false));
		_driverJoystick.L2().onTrue(TeleopCommandBuilder.resetGyro(true));

//		_driverJoystick.square().onTrue(Commands.runOnce(() -> {
//			if (RobotState.getRobotState() == RobotStates.PREPARE_SHOOT) {
//				SwerveIO.getInstance().setState(SwerveIO.getInstance().getPreviousState());
//				StateMachine.getInstance().changeRobotState(RobotStates.CLOSE);
//			} else {
//				SwerveIO.getInstance().setState(SwerveDemand.SwerveState.LOOK_AT_TARGET);
//				SwerveIO.getInstance().updateDemand(Constants.VisionConstants.getTagPose(15));
//				StateMachine.getInstance().changeRobotState(RobotStates.PREPARE_SHOOT);
//			}
//		}));

		_driverJoystick.povLeft().onTrue(Commands.runOnce(() -> {
			if (SwerveIO.getInstance().getState() == SwerveDemand.SwerveState.POSITION)
				SwerveIO.getInstance().setState(SwerveIO.getInstance().getPreviousState());
			else {
				SwerveIO.getInstance().setState(SwerveDemand.SwerveState.POSITION);
				SwerveIO.getInstance().updateDemand(Constants.VisionConstants.getTagPose(6));
			}
		}));

		_driverJoystick.povLeft().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.DRIVE_TO_AMP));
		_driverJoystick.povDown().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.DRIVE_TO_SPEAKER));
		_driverJoystick.povUp().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.DRIVE_TO_SOURCE));

		_driverJoystick.circle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.CLOSE));
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
		StateMachine.getInstance().changeRobotState(RobotStates.HOLDING_NOTE);
		TeleopCommandBuilder.resetGyro(false).schedule();
	}
}
