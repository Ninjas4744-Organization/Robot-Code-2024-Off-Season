package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NinjasLib.DataClasses.SwerveDemand;
import frc.robot.NinjasLib.Swerve.PathFollowing.LocalADStarAK;
import frc.robot.NinjasLib.Swerve.SwerveIO;

public class AutoCommandBuilder {
	public static void configureAutoBuilder() {
		Pathfinding.setPathfinder(new LocalADStarAK());

		AutoBuilder.configureHolonomic(
				RobotState::getRobotPose, // Robot pose supplier
				RobotState::setRobotPose, // Method to reset odometry (will be called if your auto has a starting
				// pose)
				SwerveIO.getInstance()::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				(drive) -> SwerveIO.getInstance()
						.updateDemand(
								drive, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				Constants.SwerveConstants.AutoConstants.kAutonomyConfig,
				// Boolean supplier that mirrors path if red alliance
				() -> DriverStation.getAlliance().get() == Alliance.Red,
				SwerveIO.getInstance() // Reference to swerve subsystem to set requirements
				);
	}

	/** Registers all auto commands to pathplanner */
	public static void registerCommands() {
		NamedCommands.registerCommand("Shoot", Shoot());
		NamedCommands.registerCommand("Intake", Intake());
	}

	/**
	 * @return final autonomy command from pathplanner
	 */
	public static Command autoCommand(String auto) {
		SwerveIO.getInstance().setState(SwerveDemand.SwerveState.VELOCITY);
		//		RobotState.setRobotState(RobotState.RobotStates.RESET);
		RobotState.setRobotState(RobotState.RobotStates.NOTE_IN_INDEXER);

		return AutoBuilder.buildAuto(auto);
	}

	public static Command Shoot() {
		return Commands.sequence(
				TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.SHOOT_SPEAKER_PREPARE),
      Commands.waitUntil(() -> RobotState.getRobotState() == RobotState.RobotStates.SHOOT_SPEAKER_READY),
				TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.SHOOT),
				Commands.waitUntil(() -> RobotState.getRobotState() == RobotState.RobotStates.NOTE_SEARCH));
	}

	public static Command Intake() {
		return Commands.sequence(
				TeleopCommandBuilder.changeRobotState(RobotState.RobotStates.INTAKE),
				Commands.waitUntil(() -> RobotState.getRobotState() == RobotState.RobotStates.NOTE_IN_INDEXER));
	}
}
