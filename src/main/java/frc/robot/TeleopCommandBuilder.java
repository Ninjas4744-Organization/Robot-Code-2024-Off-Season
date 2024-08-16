package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Swerve.Swerve;
import frc.robot.Vision.Vision;
import java.util.function.Supplier;

public class TeleopCommandBuilder {
    // public static Command EXAMPLE() {
    //   return Commands.either(
    //     System1.runCommand1(),
    //     System2.runCommand3(),
    //     () -> { return !System3.isNote(); }
    //   );
    // }

    public static Command swerveDrive(
            Supplier<Translation2d> translation, Supplier<Translation2d> rotation, boolean isLookAt) {
        return Commands.runOnce(
                () -> {
                    double lx =
                            -MathUtil.applyDeadband(translation.get().getX(), SwerveConstants.kJoystickDeadband);
                    double ly =
                            -MathUtil.applyDeadband(translation.get().getY(), SwerveConstants.kJoystickDeadband);
                    double rx =
                            -MathUtil.applyDeadband(rotation.get().getX(), SwerveConstants.kJoystickDeadband);
                    double ry =
                            -MathUtil.applyDeadband(rotation.get().getY(), SwerveConstants.kJoystickDeadband);

                    Swerve.getInstance().drive(new Translation2d(ly, lx), rx, true, true);
                    if (isLookAt) Swerve.getInstance().lookAt(new Translation2d(ry, rx), 45);
                },
                Swerve.getInstance());
    }

    public static Command resetSubsystems() {
        return Commands.parallel();
    }

    public static Command resetGyro(boolean forceZero) {
        return Commands.runOnce(
                () -> {
                    if (forceZero) RobotState.resetGyro(Rotation2d.fromDegrees(0));
                    else {
                        if (Vision.getInstance().hasTargets())
                            RobotState.resetGyro(RobotState.getRobotPose().getRotation().unaryMinus());
                        else RobotState.resetGyro(Rotation2d.fromDegrees(0));
                    }
                });
    }

    /**
      * Makes the robot auto drive to the closest tag infront of it
      *
      * @return the command that will make it drive
      */
    public static Command goToTag() {
        // return Commands.either(
        //   Commands.runOnce(() -> {
        //     Pose2d tagPose = Vision.getInstance().getClosestTag("Front").pose.toPose2d();
        //     Swerve.getInstance().goTo(tagPose, 0.25).schedule();
        //   }, Swerve.getInstance()),

        //   Commands.none(),

        //   () -> { return Vision.getInstance().hasTargets("Front"); }
        // );

        return Commands.runOnce(
                () -> {
                    if (Vision.getInstance().hasTargets("Front")) {
                        Pose2d tagPose = Vision.getInstance().getClosestTag("Front").pose.toPose2d();
                        tagPose =
                                new Pose2d(tagPose.getX(), tagPose.getY(), tagPose.getRotation().unaryMinus());
                        System.out.println(
                                "Going to tag: "
                                        + tagPose.getX()
                                        + ", "
                                        + tagPose.getY()
                                        + ", "
                                        + tagPose.getRotation().getDegrees());
                        Swerve.getInstance().goTo(tagPose, 0).schedule();
                    } else
                        Commands.print(
                                        "Cannot auto go to tag because no tags were found infront of front camera")
                                .schedule();
                },
                Swerve.getInstance());

        // return () -> {
        //   if(Vision.getInstance().hasTargets("Front")){
        //     Pose2d tagPose = Vision.getInstance().getClosestTag("Front").pose.toPose2d();
        //     return Swerve.getInstance().goTo(tagPose, 0.25);
        //   }

        //   return Commands.print("Cannot auto go to tag because no tags were found infront of front
        // camera");
        // };

        // Supplier<Command> goToTagCommand = () -> {
        //   if (Vision.getInstance().hasTargets("Front")) {
        //     Pose2d tagPose = Vision.getInstance().getClosestTag("Front").pose.toPose2d();
        //     return Swerve.getInstance().goTo(tagPose, 0.25);
        //   }
        //   return Commands.print("Cannot auto go to tag because no tags were found infront of front
        // camera");
        // };
        // return goToTagCommand.get();
    }
}
