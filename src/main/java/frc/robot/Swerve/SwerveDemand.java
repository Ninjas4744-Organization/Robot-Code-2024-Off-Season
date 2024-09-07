package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDemand {
	public enum SwerveState {
		DRIVER,
		POSITION,
		VELOCITY,
		LOCKED_AXIS,
	}

	public ChassisSpeeds driverInput = new ChassisSpeeds(0, 0, 0);
	public ChassisSpeeds velocity = new ChassisSpeeds(0, 0, 0);
	public boolean fieldRelative = true;
	public Pose2d targetPose = new Pose2d();
	public Translation2d axis = new Translation2d();
	public double phase = 0;
}
