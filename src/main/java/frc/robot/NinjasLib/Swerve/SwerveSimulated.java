package frc.robot.NinjasLib.Swerve;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;

public class SwerveSimulated extends SwerveIO {
	private ChassisSpeeds _currentChassisSpeeds = new ChassisSpeeds();
	private SlewRateLimiter _xAcceleration = new SlewRateLimiter(SwerveConstants.Simulation.kAcceleration);
	private SlewRateLimiter _yAcceleration = new SlewRateLimiter(SwerveConstants.Simulation.kAcceleration);

	@Override
	public void drive(ChassisSpeeds drive, boolean fieldRelative) {
		_currentChassisSpeeds =
				fieldRelative ? drive : ChassisSpeeds.fromRobotRelativeSpeeds(drive, RobotState.getGyroYaw());

		RobotState.setRobotPose(new Pose2d(
				RobotState.getRobotPose().getX()
						+ _xAcceleration.calculate(_currentChassisSpeeds.vxMetersPerSecond)
								* SwerveConstants.Simulation.kSimToRealSpeedConversion,
				RobotState.getRobotPose().getY()
						+ _yAcceleration.calculate(_currentChassisSpeeds.vyMetersPerSecond)
								* SwerveConstants.Simulation.kSimToRealSpeedConversion,
				RobotState.getRobotPose()
						.getRotation()
						.minus(Rotation2d.fromRadians(_currentChassisSpeeds.omegaRadiansPerSecond
								* SwerveConstants.Simulation.kSimToRealSpeedConversion))));
	}

	@Override
  public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
    return fieldRelative ? _currentChassisSpeeds : ChassisSpeeds.fromFieldRelativeSpeeds(_currentChassisSpeeds, RobotState.getGyroYaw());
	}
}
