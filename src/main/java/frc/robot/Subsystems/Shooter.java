package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Shooter extends NinjasSubsystem {
  private static Shooter _instance;

  public static Shooter getInstance(){
    if(_instance == null) _instance = new Shooter();

    return _instance;
  }

  private PIDController _pid = new PIDController(Constants.ShooterConstants.kControllerConstants.PIDFConstants.kP,
    Constants.ShooterConstants.kControllerConstants.PIDFConstants.kI,
    Constants.ShooterConstants.kControllerConstants.PIDFConstants.kD);

  @Override
  protected void setController() {
    _controller = new NinjasSparkMaxController(Constants.ShooterConstants.kControllerConstants);
  }

  @Override
  protected void setSimulationController() {
    _simulatedController = new NinjasSimulatedController(Constants.ShooterConstants.kSimulatedControllerConstants);
  }

  @Override
  protected void setFunctionMaps() {
    addFunctionToPeriodicMap(() -> {
      double shooterHeight = 0.55;
      double targetHeight = 2.25;

      double dist = RobotState.getRobotPose().getTranslation().getDistance(Constants.VisionConstants.getTagPose(15).getTranslation());
      Rotation2d angle = Rotation2d.fromRadians(Math.atan2(targetHeight - shooterHeight, dist));
      System.out.println(dist);
      double a = Math.min(Math.max(angle.getDegrees() + 30, 0), 120);

      controller().setPercent(_pid.calculate(controller().getPosition(), a));
    }, RobotState.RobotStates.HOLDING_NOTE);
  }
}
