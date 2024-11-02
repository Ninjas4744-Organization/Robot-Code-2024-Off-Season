package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.NinjasLib.DataClasses.SwerveDemand;
import frc.robot.NinjasLib.DataClasses.VisionEstimation;
import frc.robot.NinjasLib.RobotStateIO;
import frc.robot.NinjasLib.StateMachineIO;
import frc.robot.NinjasLib.Vision.VisionIO;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;
import frc.robot.Swerve.Swerve;
import frc.robot.Swerve.SwerveIO;

public class RobotContainer {
	private final CommandPS5Controller _driverJoystick;
	private final CommandPS5Controller _operatorJoystick;

	private boolean isSwerveLookAt = false;
	private boolean isSwerveBayblade = false;

	public RobotContainer() {
		RobotStateIO.setInstance(new RobotState());

		Shooter.getInstance();
		ShooterAngle.getInstance();
		StateMachineIO.setInstance(new StateMachine());
		Indexer.getInstance();
		FieldConstants.getFieldLayout();

		RobotState.getInstance().initPoseEstimator();

		AutoCommandBuilder.configureAutoBuilder();
		AutoCommandBuilder.registerCommands();

		_driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
		_operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

		Shuffleboard.getTab("Competition").addBoolean("Note", () -> RobotState.getInstance().getNoteInIndexer());
		Shuffleboard.getTab("Competition")
				.addBoolean(
						"Shooter Ready",
						() -> RobotState.getInstance().getRobotState() == RobotStates.SHOOT_SPEAKER_READY
								|| RobotState.getInstance().getRobotState() == RobotStates.SHOOT_AMP_READY);
		Shuffleboard.getTab("Competition")
				.addString("Robot State", () -> RobotState.getInstance().getRobotState().toString());
		Shuffleboard.getTab("Competition")
				.addBoolean("Can Gyro Reset", () -> VisionIO.getInstance().hasTargets());

		new Trigger(() -> RobotState.getInstance().getRobotPose().getX() <= 5 && RobotState.getInstance().getNoteInIndexer())
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
		_driverJoystick.cross().onTrue(TeleopCommandBuilder.runIfNotTestMode(TeleopCommandBuilder.changeRobotState(RobotStates.INTAKE)));

		_driverJoystick.triangle().onTrue(TeleopCommandBuilder.runIfNotTestMode(Commands.runOnce(() -> {
			if (RobotState.getInstance().getRobotPose().getX() <= 5)
				StateMachine.getInstance().changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE);
			else StateMachine.getInstance().changeRobotState(RobotStates.DELIVERY);
		})));

		_driverJoystick.square().onTrue(TeleopCommandBuilder.runIfNotTestMode(TeleopCommandBuilder.changeRobotState(RobotStates.SHOOT_AMP_PREPARE)));

		_driverJoystick.circle().onTrue(TeleopCommandBuilder.runIfNotTestMode(TeleopCommandBuilder.changeRobotState(RobotStates.CLOSE)));

		_driverJoystick.R2().onTrue(TeleopCommandBuilder.runIfNotTestMode(Commands.runOnce(
						() -> {
							StateMachine.getInstance().changeRobotState(RobotStates.SHOOT);
							StateMachine.getInstance().changeRobotState(RobotStates.OUTTAKE);
						},
						StateMachine.getInstance())));

		_driverJoystick.povUp().onTrue(TeleopCommandBuilder.runIfNotTestMode(TeleopCommandBuilder.changeRobotState(RobotStates.OOGA_BOOGA)));

		_driverJoystick.povDown().onTrue(TeleopCommandBuilder.runIfNotTestMode(Commands.runOnce(() -> {
			StateMachine.getInstance().changeRobotState(RobotStates.RESET);
			((Swerve) (SwerveIO.getInstance())).resetModulesToAbsolute();
		})));
	}

	private void configureTestBindings() {
		_operatorJoystick.triangle().whileTrue(TeleopCommandBuilder.runIfTestMode(Indexer.getInstance().runMotor(1)));
		_operatorJoystick.cross().whileTrue(TeleopCommandBuilder.runIfTestMode(Indexer.getInstance().runMotor(-1)));

		_operatorJoystick.povDown().whileTrue(TeleopCommandBuilder.runIfTestMode(ShooterAngle.getInstance().runMotor(-0.5)));
		_operatorJoystick.povUp().whileTrue(TeleopCommandBuilder.runIfTestMode(ShooterAngle.getInstance().runMotor(0.5)));

		_operatorJoystick.square().whileTrue(TeleopCommandBuilder.runIfTestMode(Shooter.getInstance().runMotor(1)));
	}

	public void periodic() {
		VisionEstimation[] estimations = VisionIO.getInstance().getVisionEstimations();
//		RobotState.getInstance().updateRobotPose(estimations);
		for (VisionEstimation estimation : estimations)
			if (estimation.pose != null)
				RobotState.getInstance().updateRobotPose(estimation);
	}

	public void resetSubsystems() {
		RobotState.getInstance().setRobotState(RobotStates.RESET);
		Shooter.getInstance().resetSubsystem();
		Indexer.getInstance().resetSubsystem();
		ShooterAngle.getInstance().resetSubsystem();
		SwerveIO.getInstance().setState(SwerveDemand.SwerveState.DEFAULT);
		SwerveIO.getInstance().drive(new ChassisSpeeds(), false);
		TeleopCommandBuilder.resetGyro(false).schedule();
	}
}
