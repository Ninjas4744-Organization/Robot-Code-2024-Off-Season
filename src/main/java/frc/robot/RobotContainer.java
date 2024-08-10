package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Swerve.Swerve;
import frc.robot.Vision.Vision;
import frc.robot.DataClasses.VisionEstimation;

public class RobotContainer {
  private CommandPS5Controller _driverJoystick;
  private Joystick _driverJoystick2;
  private CommandPS5Controller _operatorJoystick;

  public RobotContainer() {
    _driverJoystick = new CommandPS5Controller(Constants.kDriverJoystickPort);
    _driverJoystick2 = new Joystick(1);
    _operatorJoystick = new CommandPS5Controller(Constants.kOperatorJoystickPort);
    
    RobotState.initPoseEstimator();
    
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
      TeleopCommandBuilder.swerveDrive(() -> new Translation2d(_driverJoystick.getLeftX(), _driverJoystick.getLeftY()),
      () -> new Translation2d(_driverJoystick.getRightX(), _driverJoystick.getRightY()), true)
    );
      
    _driverJoystick.circle().toggleOnTrue(Commands.startEnd(
      () -> { Swerve.getInstance().setBaybladeMode(true); },
      () -> { Swerve.getInstance().setBaybladeMode(false); }
    ));

    _driverJoystick.L1().onTrue(
      Commands.parallel(
        TeleopCommandBuilder.resetGyro(false),
        
        Commands.runOnce(() -> {
          Swerve.getInstance().resetModulesToAbsolute();
        }, Swerve.getInstance())
      )
    );

    _driverJoystick.L2().onTrue(
      Commands.parallel(
        TeleopCommandBuilder.resetGyro(true),
        
        Commands.runOnce(() -> {
          Swerve.getInstance().resetModulesToAbsolute();
        }, Swerve.getInstance())
      )
    );

    _driverJoystick.R1().toggleOnTrue(Commands.startEnd(
      () -> Swerve.getInstance().setIsDriveAssist(true), 
      () -> Swerve.getInstance().setIsDriveAssist(false)
    ));

    // _driverJoystick.R2().whileTrue(TeleopCommandBuilder.goToTag());
  }

  private void configureOperatorBindings(){
    
  }

  public void periodic(){
    VisionEstimation[] estimations = Vision.getInstance().getVisionEstimations();
    
    for(VisionEstimation estimation : estimations)
      if(estimation != null)
        RobotState.updateRobotPose(estimation);
  }

  public void resetSubsystems(){
    TeleopCommandBuilder.resetGyro(false).schedule();
    TeleopCommandBuilder.resetSubsystems().schedule();
  }
}