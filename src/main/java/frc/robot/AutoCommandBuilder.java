package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Swerve.SwerveIO;
import java.util.Optional;

public class AutoCommandBuilder {
	public static void configureAutoBuilder() {
		AutoBuilder.configureHolonomic(
				RobotState::getRobotPose, // Robot pose supplier
				RobotState::setRobotPose, // Method to reset odometry (will be called if your auto has a starting
				// pose)
				SwerveIO.getInstance()::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				(drive) -> SwerveIO.getInstance()
						.drive(drive), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				Constants.AutoConstants.pathFollowerConfig,
				// Boolean supplier that mirrors path if red alliance
				() -> {
					Optional<Alliance> alliance = DriverStation.getAlliance();
					return alliance.filter(value -> value == Alliance.Red).isPresent();
				},
				SwerveIO.getInstance() // Reference to swerve subsystem to set requirements
				);
	}

	/** Registers all auto commands to pathplanner */
	public static void registerCommands() {
		// NamedCommands.registerCommand(, );
		// NamedCommands.registerCommand(, );
		// NamedCommands.registerCommand(, );
	}

	/**
	 * @return final autonomy command from pathplanner
	 */
	public static Command autoCommand(String auto) {
		return Commands.none();
	}

	// public static Command EXAMPLE() {
	//   return Commands.either(
	//     System1.runCommand1(),
	//     System2.runCommand3(),
	//     () -> { return !System3.isNote(); }
	//   );
	// }
}
