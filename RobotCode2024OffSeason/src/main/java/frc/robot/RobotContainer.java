package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Swerve.Swerve;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionEstimation;

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
      () -> new Translation2d(_driverJoystick.getRightX(), 0), false)
      // TeleopCommandBuilder.swerveDrive(() -> new Translation2d(-_driverJoystick2.getRawAxis(1), _driverJoystick2.getRawAxis(2)),
      //  () -> new Translation2d(_driverJoystick2.getRawAxis(3), 0), false)
    );
      
    _driverJoystick.R3().toggleOnTrue(Commands.startEnd(
      () -> { Swerve.getInstance().setBaybladeMode(true); },
      () -> { Swerve.getInstance().setBaybladeMode(false); }
    ));

    // new Trigger(() -> _driverJoystick2.getRawButton(1)).toggleOnTrue(Commands.startEnd(
    //   () -> { Swerve.getInstance().setBaybladeMode(true); },
    //   () -> { Swerve.getInstance().setBaybladeMode(false); }
    // ));

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
      if(estimation != null)
        RobotState.updateRobotPose(estimation);
  }

  public void resetSubsystems(){
    TeleopCommandBuilder.resetSubsystems().schedule();
  }
}