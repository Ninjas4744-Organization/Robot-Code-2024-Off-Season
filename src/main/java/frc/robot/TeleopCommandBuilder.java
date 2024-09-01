package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState.RobotStates;
import frc.robot.Swerve.Swerve;
import frc.robot.Swerve.SwerveIO;
import frc.robot.Vision.Vision;
import frc.robot.Vision.VisionIO;

import java.util.function.Supplier;

public class TeleopCommandBuilder {
	public static Command swerveDrive(
			Supplier<Translation2d> translation, Supplier<Translation2d> rotation, boolean isLookAt) {
		return Commands.runOnce(
				() -> {
					double lx = -MathUtil.applyDeadband(translation.get().getX(), SwerveConstants.kJoystickDeadband);
					double ly = -MathUtil.applyDeadband(translation.get().getY(), SwerveConstants.kJoystickDeadband);
					double rx = (RobotState.isSimulated() ? 1 : -1) * MathUtil.applyDeadband(rotation.get().getX(), SwerveConstants.kJoystickDeadband);
					double ry = -MathUtil.applyDeadband(rotation.get().getY(), SwerveConstants.kJoystickDeadband);

					SwerveIO.getInstance().drive(new Translation2d(ly, lx), rx);
					if (isLookAt) SwerveIO.getInstance().lookAt(new Translation2d(ry, rx), 45);
				},
				SwerveIO.getInstance());
	}

	public static Command resetGyro(boolean forceZero) {
		return Commands.runOnce(() -> {
			if (forceZero)
				RobotState.resetGyro(Rotation2d.fromDegrees(0));
			else {
				if (VisionIO.getInstance().hasTargets())
			 		RobotState.resetGyro(RobotState.getRobotPose().getRotation().unaryMinus());
				else
					RobotState.resetGyro(Rotation2d.fromDegrees(0));
		  	}
		});
	}

	private static Command goToTagCommand;
	/**
	 * Makes the robot auto drive to the closest tag infront of it
	 *
	 * @return the command that will make it drive
	 */
	public static Command goToTag() {
		return Commands.startEnd(
				() -> {
				if (Vision.getInstance().hasTargets("Front")) {
					Pose2d tagPose =
							Vision.getInstance().getClosestTag("Front").pose.toPose2d();
					tagPose = new Pose2d(
							tagPose.getX(),
							tagPose.getY(),
							tagPose.getRotation().unaryMinus());

					goToTagCommand = Swerve.getInstance().goTo(tagPose, 0);
					goToTagCommand.schedule();
				} else
					Commands.print("Cannot auto go to tag because no tags were found infront of front camera").schedule();
			},
			() -> CommandScheduler.getInstance().cancel(goToTagCommand), SwerveIO.getInstance());
	}

	public static Command changeRobotState(RobotStates state) {
		return Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(state), StateMachine.getInstance());
	}
}
