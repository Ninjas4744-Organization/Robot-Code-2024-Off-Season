package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;

public class Swerve extends SwerveIO {
	private SwerveModule[] _modules;
	protected SwerveDriveOdometry _odometry;

	public Swerve() {
		super();

		_modules = new SwerveModule[] {
			new SwerveModule(0, SwerveConstants.Mod0.constants),
			new SwerveModule(1, SwerveConstants.Mod1.constants),
			new SwerveModule(2, SwerveConstants.Mod2.constants),
			new SwerveModule(3, SwerveConstants.Mod3.constants)
		};

		_odometry = new SwerveDriveOdometry(
				SwerveConstants.kSwerveKinematics, RobotState.getGyroYaw(), getModulePositions());
		resetOdometry(new Pose2d());
	}

	@Override
	public void drive(ChassisSpeeds drive, boolean fieldRelative) {
		SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
			fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(drive, RobotState.getGyroYaw()) : drive);
		setModuleStates(swerveModuleStates, SwerveConstants.kOpenLoop);
	}

	/**
	 * Sets the modules to the given states
	 *
	 * @param desiredStates - the wanted state for each module
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

		for (SwerveModule mod : _modules) mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
	}

	/**
	 * @return array of module states
	 */
	public SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (SwerveModule mod : _modules) {
			states[mod.moduleNumber] = mod.getState();
		}
		return states;
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
	}

	/**
	 * Resets the swerve odometry to the given pose, so it thinks the robot is at that pose
	 *
	 * @param pose - the pose to reset the odometry to
	 */
	public void resetOdometry(Pose2d pose) {
		_odometry.resetPosition(RobotState.getGyroYaw(), getModulePositions(), pose);
	}

	/**
	 * @return current pose of the robot according to odometry
	 */
	public Pose2d getOdometryPosition() {
		return _odometry.getPoseMeters();
	}

	/** Sets the swerve modules rotation, so it makes an X shape and makes the swerve immovable */
	public void setWheelsToX() {
		setModuleStates(
				new SwerveModuleState[] {
					// front left
					new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
					// front right
					new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
					// back left
					new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
					// back right
					new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
				},
				SwerveConstants.kOpenLoop);
	}

	@Override
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (SwerveModule mod : _modules) positions[mod.moduleNumber] = mod.getPosition();
		return positions;
	}

	/**
	 * Resets the swerve modules to their absolute encoders to fix problems with a module no knowing
	 * where it is
	 */
	public void resetModulesToAbsolute() {
		System.out.println("---------------Reseting modules to absolute---------------");
		for (SwerveModule mod : _modules) mod.resetToAbsolute();
		System.out.println("---------------Reseting modules to absolute---------------");
	}

	@Override
	public void periodic() {
		super.periodic();
		_odometry.update(RobotState.getGyroYaw(), getModulePositions());
		RobotState.updateRobotPose(getModulePositions());
	}
}
