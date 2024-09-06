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

	public ChassisSpeeds chassisSpeeds;
	public boolean fieldRelative;

	public Pose2d targetPose;

	public Translation2d axis;
	public double phase;

	public SwerveState state;

	public SwerveDemand(ChassisSpeeds driverInput) {
		this.chassisSpeeds = driverInput;

		state = SwerveState.DRIVER;
	}

	public SwerveDemand(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
		this.chassisSpeeds = chassisSpeeds;
		this.fieldRelative = fieldRelative;

		state = SwerveState.VELOCITY;
	}

	public SwerveDemand(Pose2d targetPose) {
		this.targetPose = targetPose;

		state = SwerveState.POSITION;
	}

	public SwerveDemand(Translation2d axis, double phase) {
		this.axis = axis;
		this.phase = phase;

		state = SwerveState.LOCKED_AXIS;
	}
}
