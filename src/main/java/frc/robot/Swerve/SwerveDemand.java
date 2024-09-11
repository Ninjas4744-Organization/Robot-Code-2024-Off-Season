package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDemand {
	public enum SwerveState {
		DEFAULT,
		POSITION,
		VELOCITY,
		LOCKED_AXIS,
		LOOK_AT_ANGLE,
		LOOK_AT_TARGET,
		BAYBLADE
	}

	public ChassisSpeeds driverInput = new ChassisSpeeds(0, 0, 0);
	public Translation2d lookAtTranslation = new Translation2d();
	public ChassisSpeeds velocity = new ChassisSpeeds(0, 0, 0);
	public boolean fieldRelative = true;
	public Pose2d targetPose = new Pose2d();
	public Rotation2d angle = new Rotation2d();
	public double phase = 0;
	public boolean isXDriverInput = false;
}
