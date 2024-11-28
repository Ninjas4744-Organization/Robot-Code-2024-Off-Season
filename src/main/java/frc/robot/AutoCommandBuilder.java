package frc.robot;

import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;
import frc.robot.Swerve.PathFollowing.LocalADStarAK;
import frc.robot.Swerve.SwerveIO;

public class AutoCommandBuilder {
	public static void configureAutoBuilder() {
		Pathfinding.setPathfinder(new LocalADStarAK());

		AutoBuilder.configureHolonomic(
			() -> new Pose2d(
				RobotState.getInstance().getRobotPose().getX(),
				RobotState.getInstance().getRobotPose().getY() * (RobotState.getInstance().getAlliance() == Alliance.Red ? -1 : 1),
				RobotState.getInstance().getRobotPose().getRotation().rotateBy(Rotation2d.fromDegrees(RobotState.getInstance().getAlliance() == Alliance.Red ? 180 : 0))
			), // Robot pose supplier

			(pose) -> {
				RobotState.getInstance().setRobotPose(new Pose2d(
					pose.getX(),
					pose.getY() * (RobotState.getInstance().getAlliance() == Alliance.Red ? -1 : 1),
					pose.getRotation().rotateBy(Rotation2d.fromDegrees(RobotState.getInstance().getAlliance() == Alliance.Red ? 180 : 0))
				));
				RobotState.getInstance().resetGyro(pose.getRotation());
			}, // Method to reset odometry (will be called if your auto has a starting pose)

			() -> SwerveIO.getInstance().getChassisSpeeds(false), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE

			(drive) -> SwerveIO.getInstance().drive(drive, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

			SwerveConstants.AutoConstants.kAutonomyConfig, //Autonomy config

			() -> false/*RobotState.getInstance().getAlliance() == Alliance.Red*/, // Boolean supplier that mirrors path if red alliance

			SwerveIO.getInstance() // Reference to swerve subsystem to set requirements
		);
	}

	/** Registers all auto commands to pathplanner */
	public static void registerCommands() {
		NamedCommands.registerCommand(
			"Prepare Shoot", prepareShoot());

		NamedCommands.registerCommand(
				"Wait Shoot Ready",
				Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.SHOOT_SPEAKER_READY));

		NamedCommands.registerCommand("Shoot", shoot());

		NamedCommands.registerCommand(
				"Full Shoot",
				Commands.sequence(
					prepareShoot(),
						Commands.waitUntil(
								() -> RobotState.getInstance().getRobotState() == RobotStates.SHOOT_SPEAKER_READY),
					shoot(),
						Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.NOTE_SEARCH)));

		NamedCommands.registerCommand("Intake", TeleopCommandBuilder.changeRobotState(RobotStates.INTAKE));

		NamedCommands.registerCommand("Wait Note", Commands.waitUntil(() -> RobotState.getInstance().getRobotState() == RobotStates.NOTE_IN_INDEXER));

		NamedCommands.registerCommand(
				"Reset",
				Commands.sequence(
						TeleopCommandBuilder.changeRobotState(RobotStates.RESET),
						Commands.waitUntil(() -> ShooterAngle.getInstance().isResetted()
								&& Indexer.getInstance().isResetted()
								&& Shooter.getInstance().isResetted())));

		NamedCommands.registerCommand("Stop", Commands.runOnce(() -> SwerveIO.getInstance().drive(new ChassisSpeeds(), false)));

		NamedCommands.registerCommand("Jiggle Back Right", jiggleBack(new Translation2d(-1, 0), new Translation2d(0, -1)));
		NamedCommands.registerCommand("Jiggle Back Left", jiggleBack(new Translation2d(-1, 0), new Translation2d(0, 1)));

		NamedCommands.registerCommand("Print 1", Commands.print("11111111111111111111"));

		NamedCommands.registerCommand("Print 2", Commands.print("22222222222222222222"));
	}

	private static Command prepareShoot() {
		return Commands.runOnce(() -> {
			StateMachine.getInstance().changeRobotState(RobotStates.SHOOT_SPEAKER_PREPARE);
			SwerveIO.getInstance().updateDemand(FieldConstants.getTagPose(FieldConstants.getSpeakerTag().ID).toPose2d());
			SwerveIO.getInstance().setState(SwerveDemand.SwerveState.LOOK_AT_TARGET);
		});
	}

	private static Command shoot() {
		return Commands.runOnce(() -> {
			StateMachine.getInstance().changeRobotState(RobotStates.SHOOT);
			SwerveIO.getInstance().setState(SwerveDemand.SwerveState.AUTONOMY);
		});
	}

	static double t = 0;

	private static Command jiggleBack(Translation2d dir, Translation2d jiggleDir) {
		return Commands.run(() -> {
			double speed = 0.15 * SwerveConstants.maxSpeed;
			double jiggleSpeed = Math.cos(t - 0.5) * 0.45 * SwerveConstants.maxSpeed;
			SwerveIO.getInstance().drive(new ChassisSpeeds(
				dir.getX() * speed + jiggleDir.getX() * jiggleSpeed,
				dir.getY() * speed + jiggleDir.getY() * jiggleSpeed,
				0
			), false);

			t += 0.02 * Math.PI * 2;
		}).until(() -> RobotState.getInstance().getNoteInIndexer()).andThen(Commands.runOnce(() -> {
			SwerveIO.getInstance().drive(new ChassisSpeeds(), false);
			t = 0;
		}));
	}

	/**
	 * @return final autonomy command from pathplanner
	 */
	public static Command autoCommand(String auto) {
		SwerveIO.getInstance().setState(SwerveDemand.SwerveState.AUTONOMY);
		RobotState.getInstance().setRobotState(RobotStates.NOTE_IN_INDEXER);

		return AutoBuilder.buildAuto(auto);
	}
}
