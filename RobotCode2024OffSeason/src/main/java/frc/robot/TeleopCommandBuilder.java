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
  // public static Command PLACEHOLDER() {
  //   return Commands.either(
  //     System1.runCommand1(),
  //     System2.runCommand3(),
  //     () -> { return !System3.isNote(); }
  //   );
  // }

  public static Command swerveDrive(Swerve swerve, CommandPS5Controller joystick, boolean isLookAt, Vision vision) {
    return Commands.sequence(
      Commands.run(() -> {
        double lx = MathUtil.applyDeadband(joystick.getLeftX(), Constants.Swerve.kJoystickDeadband);
        double ly = MathUtil.applyDeadband(joystick.getLeftY(), Constants.Swerve.kJoystickDeadband);
        double rx = MathUtil.applyDeadband(joystick.getRightX(), Constants.Swerve.kJoystickDeadband);

        swerve.drive(
          new Translation2d(ly, lx), rx, true, false);
      }, swerve),

      Commands.either(
        swerveLookAt(swerve, joystick), 
        Commands.none(), 
        () -> isLookAt),

      Commands.run(() -> {
        VisionEstimation[] data = vision.getEstimationsData();
        
        for(VisionEstimation estimation : data)
          swerve.addVisionEstimation(estimation);
      }, swerve, vision)
    );
  }

  public static Command swerveLookAt(Swerve swerve, CommandPS5Controller joystick) {
    return Commands.run(() -> {
        double rx = MathUtil.applyDeadband(joystick.getRightX(), Constants.Swerve.kJoystickDeadband);
        double ry = MathUtil.applyDeadband(joystick.getRightY(), Constants.Swerve.kJoystickDeadband);

        swerve.lookAt(new Translation2d(rx, ry), 45);
      }, swerve);
  }

  public static Command resetSubsystems(Vision vision) {
    return Commands.parallel(
      Commands.run(() -> {
        if(vision.hasTargets())
          RobotState.resetGyro(RobotState.getRobotPose().getRotation());
        else
          RobotState.resetGyro(Rotation2d.fromDegrees(0));
      }),

      Commands.none()
    );
  }

  public static Command goToTag(Swerve swerve, Vision vision){
    Pose2d tagPose = vision.getClosestTag("front").pose.toPose2d();
    return swerve.goTo(tagPose, 0.25);
  }
}