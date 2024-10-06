package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.NinjasLib.DataClasses.VisionEstimation;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;
import frc.robot.NinjasLib.Swerve.SwerveIO;
import frc.robot.NinjasLib.Vision.VisionIO;

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

		Climber.getInstance();
		Indexer.getInstance();
		Shooter.getInstance();
		ShooterAngle.getInstance();

		configureBindings();
	}

	private void configureBindings() {
		_driverJoystick.circle().toggleOnTrue(Commands.runOnce(()->StateMachine.getInstance().changeRobotState(RobotStates.SHOOT)));
		_driverJoystick.circle().toggleOnTrue(Commands.runOnce(()->StateMachine.getInstance().changeRobotState(RobotStates.CLIMB)));
		_driverJoystick.circle().toggleOnTrue(Commands.runOnce(()->StateMachine.getInstance().changeRobotState(RobotStates.NOTE_SEARCH)));
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

		_driverJoystick.square().onTrue(Commands.runOnce(() -> isSwerveBayblade = !isSwerveBayblade));
		_driverJoystick.triangle().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));

		_driverJoystick.L1().onTrue(TeleopCommandBuilder.resetGyro(false));
		_driverJoystick.L2().onTrue(TeleopCommandBuilder.resetGyro(true));

		_driverJoystick.povLeft().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.DRIVE_TO_AMP));
		_driverJoystick.povUp().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.DRIVE_TO_SOURCE));
	}

	private void configureOperatorBindings() {
		_driverJoystick.cross().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE));
		_driverJoystick.circle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.CLOSE));
	}

	public void periodic() {
		VisionEstimation[] estimations = VisionIO.getInstance().getVisionEstimations();

		for (VisionEstimation estimation : estimations) if (estimation != null) RobotState.updateRobotPose(estimation);
	}

	public void resetSubsystems() {
		RobotState.setRobotPose(new Pose2d());
		RobotState.setRobotState(RobotStates.RESET);
		TeleopCommandBuilder.resetGyro(false).schedule();
	}
}
