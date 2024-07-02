package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Swerve.Swerve;
import frc.robot.Vision.Vision;

public class RobotContainer {
  // Subsystems
  private Swerve _swerve;
  private Vision _vision;

  // Misc
  private CommandPS5Controller _driverJoystick;
  private CommandPS5Controller _operatorJoystick;

  public RobotContainer() {
    _driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
    _operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

    _swerve = new Swerve();
    _vision = new Vision(new String[]{"Front", "BackLeft"});

    AutoCommandBuilder.registerCommands();

    configureBindings();
  }

  private void configureBindings() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings(){
    _swerve.setDefaultCommand(
      TeleopCommandBuilder.swerveDrive(_swerve, _driverJoystick, false, _vision)
    );

    _driverJoystick.R3().toggleOnTrue(Commands.startEnd(
      () -> { _swerve.setBaybladeMode(true); },
      () -> { _swerve.setBaybladeMode(false); }
    ));

    _driverJoystick.L1().onTrue(
      Commands.runOnce(() -> {
        RobotState.resetGyro(Rotation2d.fromDegrees(0));
        _swerve.resetModulesToAbsolute();
      }, _swerve)
    );
  }

  private void configureOperatorBindings(){
    
  }

  public void periodic(){
    
  }

  public void resetSubsystems(){
    TeleopCommandBuilder.resetSubsystems(_vision).schedule();
  }
}