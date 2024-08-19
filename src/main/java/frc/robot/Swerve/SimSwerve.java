package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.RobotState;

public class SimSwerve extends SwerveIO {

	private Pose2d _currentPose = new Pose2d();

	@Override
	public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
		_currentPose = new Pose2d(
				_currentPose.getX() + translation.getX() * 0.1,
				_currentPose.getY() + translation.getY() * 0.1,
				_currentPose.getRotation().plus(Rotation2d.fromDegrees(rotation * 10)));
	}

	@Override
	public SwerveModulePosition[] getModulePositions() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getModulePositions'");
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getChassisSpeeds'");
	}

	@Override
	public void periodic() {
		RobotState.setRobotPose(_currentPose);
	}
}
