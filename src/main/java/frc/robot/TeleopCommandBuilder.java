package frc.robot;

import com.ninjas4744.NinjasLib.Vision.VisionIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Swerve.SwerveIO;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class TeleopCommandBuilder {
	public static Command swerveDrive(
			Supplier<Translation2d> translation,
			Supplier<Translation2d> rotation,
			BooleanSupplier isLookAt,
			BooleanSupplier isBayblade) {
		return Commands.runOnce(
				() -> {
					double lx = -MathUtil.applyDeadband(translation.get().getX(), SwerveConstants.kJoystickDeadband);
					double ly = -MathUtil.applyDeadband(translation.get().getY(), SwerveConstants.kJoystickDeadband);
					double rx = (RobotState.getInstance().isSimulated() ? 1 : -1)
							* MathUtil.applyDeadband(rotation.get().getX(), SwerveConstants.kJoystickDeadband);
					double ry = -MathUtil.applyDeadband(rotation.get().getY(), SwerveConstants.kJoystickDeadband);

					double finalRotation =
							rx * SwerveConstants.maxAngularVelocity * SwerveConstants.kRotationSpeedFactor;

					if (isLookAt.getAsBoolean())
						finalRotation = SwerveIO.getInstance().lookAt(new Translation2d(ry, rx), 45);

					if (isBayblade.getAsBoolean()) finalRotation = SwerveConstants.maxAngularVelocity;

					SwerveIO.getInstance().updateDemand(new ChassisSpeeds(ly, lx, finalRotation));
				},
				SwerveIO.getInstance());
	}

	public static Command resetGyro(boolean forceZero) {
		return Commands.runOnce(() -> {
			if (forceZero) RobotState.getInstance().resetGyro(Rotation2d.fromDegrees(0));
			else {
				if (VisionIO.getInstance().hasTargets())
					RobotState.getInstance().resetGyro(RobotState.getInstance().getRobotPose().getRotation());
				else RobotState.getInstance().resetGyro(Rotation2d.fromDegrees(0));
			}
		});
	}

	public static Command changeRobotState(RobotStates state) {
		return Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(state), StateMachine.getInstance());
	}

	public static Command runIfTestMode(Command command) {
		return Commands.either(
			command,
			Commands.none(),
			() -> RobotState.getInstance().getRobotState() == RobotStates.TESTING
		);
	}

	public static Command runIfNotTestMode(Command command) {
		return Commands.either(
			command,
			Commands.none(),
			() -> RobotState.getInstance().getRobotState() != RobotStates.TESTING
		);
	}
}
