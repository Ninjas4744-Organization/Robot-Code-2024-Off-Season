package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Swerve.Swerve;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionEstimation;

public class RobotContainer {
  private CommandPS5Controller _driverJoystick;
  private CommandPS5Controller _operatorJoystick;

  public RobotContainer() {
    _driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
    _operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);

    AutoCommandBuilder.configureAutoBuilder();
    AutoCommandBuilder.registerCommands();

    configureBindings();
  }

  private void configureBindings() {
    configureDriverBindings();
    configureOperatorBindings();
  }

  private void configureDriverBindings(){
    Swerve.getInstance().setDefaultCommand(
      TeleopCommandBuilder.swerveDrive(_driverJoystick, false)
    );

    _driverJoystick.R3().toggleOnTrue(Commands.startEnd(
      () -> { Swerve.getInstance().setBaybladeMode(true); },
      () -> { Swerve.getInstance().setBaybladeMode(false); }
    ));

    _driverJoystick.L1().onTrue(
      Commands.runOnce(() -> {
        TeleopCommandBuilder.resetGyro();
        Swerve.getInstance().resetModulesToAbsolute();
      }, Swerve.getInstance())
    );

    _driverJoystick.R2().whileTrue(TeleopCommandBuilder.goToTag());
  }

  private void configureOperatorBindings(){
    
  }

  public void periodic(){
    VisionEstimation[] estimations = Vision.getInstance().getVisionEstimations();
    
    for(VisionEstimation estimation : estimations)
      RobotState.updateRobotPose(estimation);
  }

  public void resetSubsystems(){
    TeleopCommandBuilder.resetSubsystems().schedule();
  }
}