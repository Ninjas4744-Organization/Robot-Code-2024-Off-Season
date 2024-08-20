package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;

public class SwerveSimulated extends SwerveIO {
	private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
	private Rotation2d rotation = RobotState.getRobotPose().getRotation();

	@Override
	public void drive(ChassisSpeeds drive, boolean fieldRelative) {
		currentChassisSpeeds =
				!fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(drive, RobotState.getGyroYaw()) : drive;

		rotation = rotation.minus(Rotation2d.fromRadians(
				currentChassisSpeeds.omegaRadiansPerSecond * SwerveConstants.Simulation.kSimToRealSpeedConversion));
		RobotState.setRobotPose(new Pose2d(
				RobotState.getRobotPose().getX()
						+ currentChassisSpeeds.vxMetersPerSecond * SwerveConstants.Simulation.kSimToRealSpeedConversion,
				RobotState.getRobotPose().getY()
						+ currentChassisSpeeds.vyMetersPerSecond * SwerveConstants.Simulation.kSimToRealSpeedConversion,
				SwerveConstants.kInvertGyro ? rotation.unaryMinus() : rotation));
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return currentChassisSpeeds;
	}
}
