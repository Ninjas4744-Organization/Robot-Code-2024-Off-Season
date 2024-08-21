package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;

public class SwerveSimulated extends SwerveIO {
	private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();

	@Override
	public void drive(ChassisSpeeds drive, boolean fieldRelative) {
		currentChassisSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(drive, RobotState.getGyroYaw()) : drive;
		currentChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(currentChassisSpeeds, RobotState.getSimulationRobotPose().getRotation());
		
		RobotState.setRobotPose(new Pose2d(
				RobotState.getRobotPose().getX()
						+ currentChassisSpeeds.vxMetersPerSecond * SwerveConstants.Simulation.kSimToRealSpeedConversion,
				RobotState.getRobotPose().getY()
						+ currentChassisSpeeds.vyMetersPerSecond * SwerveConstants.Simulation.kSimToRealSpeedConversion,
				RobotState.getRobotPose().getRotation().minus(Rotation2d.fromRadians(currentChassisSpeeds.omegaRadiansPerSecond * SwerveConstants.Simulation.kSimToRealSpeedConversion))
		));

		RobotState.moveSimulationRobotPose(new Transform2d(
			currentChassisSpeeds.vxMetersPerSecond * SwerveConstants.Simulation.kSimToRealSpeedConversion,
			currentChassisSpeeds.vyMetersPerSecond * SwerveConstants.Simulation.kSimToRealSpeedConversion,
			Rotation2d.fromRadians((SwerveConstants.kInvertGyro ? currentChassisSpeeds.omegaRadiansPerSecond : -currentChassisSpeeds.omegaRadiansPerSecond) * SwerveConstants.Simulation.kSimToRealSpeedConversion)
		));
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return currentChassisSpeeds;
	}
}
