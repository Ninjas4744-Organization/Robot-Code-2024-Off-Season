package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NinjasLib.DataClasses.SwerveDemand;
import frc.robot.NinjasLib.Swerve.PathFollowing.LocalADStarAK;
import frc.robot.NinjasLib.Swerve.SwerveIO;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;

public class AutoCommandBuilder {
	public static void configureAutoBuilder() {
		Pathfinding.setPathfinder(new LocalADStarAK());

		AutoBuilder.configureHolonomic(
			() -> new Pose2d(
				RobotState.getRobotPose().getX(),
				RobotState.getRobotPose().getY(),
				RobotState.getRobotPose().getRotation().rotateBy(Rotation2d.fromDegrees(RobotState.getAlliance() == Alliance.Red ? 180 : 0))
			), // Robot pose supplier
//			RobotState::getRobotPose,
//			(pose) -> {
//				RobotState.setRobotPose(pose);
//				RobotState.resetGyro(pose.getRotation());
//			}, // Method to reset odometry (will be called if your auto has a starting
			RobotState::setRobotPose,
				// pose)
			() -> SwerveIO.getInstance().getChassisSpeeds(false), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				(drive) -> SwerveIO.getInstance()
					.drive(drive, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				Constants.SwerveConstants.AutoConstants.kAutonomyConfig,
				// Boolean supplier that mirrors path if red alliance
			() -> RobotState.getAlliance() == Alliance.Red,
				SwerveIO.getInstance() // Reference to swerve subsystem to set requirements
				);
	}

	/** Registers all auto commands to pathplanner */
	public static void registerCommands() {
		NamedCommands.registerCommand(
				"Prepare Shoot", TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.SHOOT_SPEAKER_PREPARE));
		NamedCommands.registerCommand(
				"Wait Shoot Ready",
				Commands.waitUntil(() -> RobotState.getRobotState() == RobotState.RobotStates.SHOOT_SPEAKER_READY));
		NamedCommands.registerCommand("Shoot", TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.SHOOT));
		NamedCommands.registerCommand(
				"Full Shoot",
				Commands.sequence(
						TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.SHOOT_SPEAKER_PREPARE),
						Commands.waitUntil(
								() -> RobotState.getRobotState() == RobotState.RobotStates.SHOOT_SPEAKER_READY),
						TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.SHOOT),
						Commands.waitUntil(() -> RobotState.getRobotState() == RobotState.RobotStates.NOTE_SEARCH)));
		NamedCommands.registerCommand("Intake", TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.INTAKE));
		NamedCommands.registerCommand("Wait Note", Commands.waitUntil(() -> RobotState.getRobotState() == RobotState.RobotStates.NOTE_IN_INDEXER));
		NamedCommands.registerCommand(
				"Reset",
				Commands.sequence(
						TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.RESET),
						Commands.waitUntil(() -> ShooterAngle.getInstance().isResetted()
								&& Indexer.getInstance().isResetted()
								&& Shooter.getInstance().isResetted())));
		NamedCommands.registerCommand("Stop", Commands.runOnce(() -> SwerveIO.getInstance().drive(new ChassisSpeeds(), false)));
		NamedCommands.registerCommand("Print 1", Commands.print("Start"));
		NamedCommands.registerCommand("Print 2", Commands.print("Note In Indexer"));
	}

	/**
	 * @return final autonomy command from pathplanner
	 */
	public static Command autoCommand(String auto) {
		SwerveIO.getInstance().setState(SwerveDemand.SwerveState.AUTONOMY);
		RobotState.setRobotState(RobotState.RobotStates.NOTE_IN_INDEXER);

		return AutoBuilder.buildAuto(auto);
	}
}
