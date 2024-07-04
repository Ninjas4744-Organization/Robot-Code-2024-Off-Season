package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Swerve.Swerve;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionEstimation;

public class TeleopCommandBuilder {
  // public static Command EXAMPLE() {
  //   return Commands.either(
  //     System1.runCommand1(),
  //     System2.runCommand3(),
  //     () -> { return !System3.isNote(); }
  //   );
  // }

  public static Command swerveDrive(Translation2d translation, Translation2d rotation, boolean isLookAt) {
    return Commands.sequence(
      Commands.runOnce(() -> {
        double lx = -MathUtil.applyDeadband(translation.getY(), Constants.Swerve.kJoystickDeadband);
        double ly = -MathUtil.applyDeadband(translation.getX(), Constants.Swerve.kJoystickDeadband);
        double rx = -MathUtil.applyDeadband(rotation.getX(), Constants.Swerve.kJoystickDeadband);

        Swerve.getInstance().drive(
          new Translation2d(ly, lx), rx, true, false);
      }, Swerve.getInstance()),

      Commands.either(
        swerveLookAt(rotation), 
        Commands.none(),
        () -> isLookAt)
    );
  }

  public static Command swerveLookAt(Translation2d rotation) {
    return Commands.run(() -> {
        double rx = MathUtil.applyDeadband(rotation.getX(), Constants.Swerve.kJoystickDeadband);
        double ry = MathUtil.applyDeadband(rotation.getY(), Constants.Swerve.kJoystickDeadband);

        Swerve.getInstance().lookAt(new Translation2d(rx, ry), 45);
      }, Swerve.getInstance());
  }

  public static Command resetSubsystems() {
    return Commands.parallel(
      resetGyro()
    );
  }

  public static Command resetGyro(){
    return Commands.run(() -> {
      if(Vision.getInstance().hasTargets())
        RobotState.resetGyro(RobotState.getRobotPose().getRotation());
      else
        RobotState.resetGyro(Rotation2d.fromDegrees(0));
    });
  }

  /**
   * Makes the robot auto drive to the closest tag infront of it
   * @return the command that will make it drive
   */
  public static Command goToTag(){
    return Commands.either(
      Commands.runOnce(() -> {
        Pose2d tagPose = Vision.getInstance().getClosestTag("Front").pose.toPose2d();
        Swerve.getInstance().goTo(tagPose, 0.25).schedule();
      }, Swerve.getInstance()),
      Commands.none(),
      () -> { return Vision.getInstance().hasTargets("Front"); }
    );
  }
}