package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NinjasLib.DataClasses.VisionEstimation;
import frc.robot.NinjasLib.Swerve.SwerveIO;
import frc.robot.NinjasLib.Vision.VisionIO;
import frc.robot.RobotState.RobotStates;
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

		Shuffleboard.getTab("Competition").addBoolean("Note", RobotState::getNoteInIndexer);
		Shuffleboard.getTab("Competition")
				.addBoolean(
						"Shooter Ready",
						() -> RobotState.getRobotState() == RobotStates.SHOOT_SPEAKER_READY
								|| RobotState.getRobotState() == RobotStates.SHOOT_AMP_READY);
		Shuffleboard.getTab("Competition")
				.addString("Robot State", () -> RobotState.getRobotState().toString());
		Shuffleboard.getTab("Competition")
				.addBoolean("Can Gyro Reset", () -> VisionIO.getInstance().hasTargets());

		new Trigger(() -> RobotState.getRobotPose().getX() <= 5 && RobotState.getNoteInIndexer())
				.onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE));

		configureBindings();
	}

	private void configureBindings() {
		StateMachine.getInstance().setTriggerForSimulationTesting(_driverJoystick.R2());

		configureTestBindings();
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

		_driverJoystick.R1().onTrue(Commands.runOnce(() -> isSwerveLookAt = !isSwerveLookAt));
		_driverJoystick.R2().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT));

		_driverJoystick.L1().onTrue(TeleopCommandBuilder.resetGyro(false));
		_driverJoystick.L2().onTrue(TeleopCommandBuilder.resetGyro(true));
	}

	private void configureOperatorBindings() {
		_driverJoystick.cross().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.INTAKE));

		_driverJoystick.triangle().onTrue(Commands.runOnce(() -> {
			if (RobotState.getRobotPose().getX() <= 5)
				StateMachine.getInstance().changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE);
			else StateMachine.getInstance().changeRobotState(RobotStates.DELIVERY);
		}));

		_driverJoystick.square().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_AMP_PREPARE));

		_driverJoystick.circle().onTrue(TeleopCommandBuilder.changeRobotState(RobotStates.CLOSE));

		_driverJoystick
				.R2()
				.onTrue(Commands.runOnce(
						() -> {
							StateMachine.getInstance().changeRobotState(RobotStates.SHOOT);
							StateMachine.getInstance().changeRobotState(RobotStates.OUTTAKE);
						},
						StateMachine.getInstance()));
	}

	private void configureTestBindings() {
		_driverJoystick.triangle().whileTrue(TeleopCommandBuilder.runIfTestMode(Indexer.getInstance().runMotor(1)));
		_driverJoystick.cross().whileTrue(TeleopCommandBuilder.runIfTestMode(Indexer.getInstance().runMotor(-1)));

		_driverJoystick.povDown().whileTrue(TeleopCommandBuilder.runIfTestMode(ShooterAngle.getInstance().runMotor(-0.5)));
		_driverJoystick.povUp().whileTrue(TeleopCommandBuilder.runIfTestMode(ShooterAngle.getInstance().runMotor(0.5)));

		_driverJoystick.square().whileTrue(TeleopCommandBuilder.runIfTestMode(Shooter.getInstance().runMotor(1)));
	}

	public void periodic() {
		SmartDashboard.putNumber(
				"Distance",
				RobotState.getRobotPose()
						.getTranslation()
						.plus(Constants.ShooterAngleConstants.kShooterPose.toTranslation2d())
						.getDistance(Constants.ShooterAngleConstants.getSpeakerHolePose()
								.toPose2d()
								.getTranslation()));

		VisionEstimation[] estimations = VisionIO.getInstance().getVisionEstimations();
		RobotState.updateRobotPose(estimations);
//		for(VisionEstimation estimation : estimations)
//			if(estimation.pose != null)
//				RobotState.updateRobotPose(estimation);
	}

	public void resetSubsystems() {
		RobotState.setRobotState(RobotStates.RESET);
		Shooter.getInstance().resetSubsystem();
		Indexer.getInstance().resetSubsystem();
		ShooterAngle.getInstance().resetSubsystem();

		TeleopCommandBuilder.resetGyro(false)
				.schedule();
	}
}
