package frc.robot.Swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;

public class SwerveSimulated extends SwerveIO {
	private ChassisSpeeds _currentChassisSpeeds = new ChassisSpeeds();
	private final SlewRateLimiter _xAcceleration = new SlewRateLimiter(SwerveConstants.Simulation.kAcceleration);
	private final SlewRateLimiter _yAcceleration = new SlewRateLimiter(SwerveConstants.Simulation.kAcceleration);

	@Override
	public void drive(ChassisSpeeds drive, boolean fieldRelative) {
		_currentChassisSpeeds =
				fieldRelative ? drive : ChassisSpeeds.fromRobotRelativeSpeeds(drive, RobotState.getInstance().getGyroYaw());

		RobotState.getInstance().setRobotPose(new Pose2d(
				RobotState.getInstance().getRobotPose().getX()
						+ _xAcceleration.calculate(_currentChassisSpeeds.vxMetersPerSecond)
								* SwerveConstants.Simulation.kSimToRealSpeedConversion,
				RobotState.getInstance().getRobotPose().getY()
						+ _yAcceleration.calculate(_currentChassisSpeeds.vyMetersPerSecond)
								* SwerveConstants.Simulation.kSimToRealSpeedConversion,
				RobotState.getInstance().getRobotPose()
						.getRotation()
						.minus(Rotation2d.fromRadians(_currentChassisSpeeds.omegaRadiansPerSecond
								* SwerveConstants.Simulation.kSimToRealSpeedConversion))));
	}

	@Override
  	public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
    	return fieldRelative ? _currentChassisSpeeds : ChassisSpeeds.fromFieldRelativeSpeeds(_currentChassisSpeeds, RobotState.getInstance().getGyroYaw());
	}
}
