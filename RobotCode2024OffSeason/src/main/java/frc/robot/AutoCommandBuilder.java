package frc.robot;

import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutoCommandBuilder {
  // public static Command PLACEHOLDER() {
  //   return Commands.either(
  //     System1.runCommand1(),
  //     System2.runCommand3(),
  //     () -> { return !System3.isNote(); }
  //   );
  // }

  /**
   * Registers all auto commands to pathplanner
   */
  public static void registerCommands() {
    // NamedCommands.registerCommand(, );
    // NamedCommands.registerCommand(, );
    // NamedCommands.registerCommand(, );
  }

  public static Command autoCommand(String auto) {
    return Commands.none();
  }
}