package frc.robot.Swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;

public class Swerve extends SwerveIO {
	private final SwerveModule[] _modules;
//	SlewRateLimiter _xRateLimiter;
//	SlewRateLimiter _yRateLimiter;

	public Swerve() {
		super();

		_modules = new SwerveModule[] {
			new SwerveModule(0, SwerveConstants.Mod0.constants),
			new SwerveModule(1, SwerveConstants.Mod1.constants),
			new SwerveModule(2, SwerveConstants.Mod2.constants),
			new SwerveModule(3, SwerveConstants.Mod3.constants)
		};

//		_xRateLimiter = new SlewRateLimiter(6.5, -Double.MAX_VALUE, 0);
//		_yRateLimiter = new SlewRateLimiter(6.5, -Double.MAX_VALUE, 0);
	}

	@Override
	public void drive(ChassisSpeeds drive, boolean fieldRelative) {
		//		Shuffleboard.getTab("Swerve").add("Drive X", drive.vxMetersPerSecond);
		//		Shuffleboard.getTab("Swerve").add("Drive Y", drive.vyMetersPerSecond);
		//		Shuffleboard.getTab("Swerve").add("Drive Omega", drive.omegaRadiansPerSecond);
		//		Shuffleboard.getTab("Swerve").add("Drive Field Relative", fieldRelative);

//		drive = new ChassisSpeeds(
//			_xRateLimiter.calculate(drive.vxMetersPerSecond),
//			_yRateLimiter.calculate(drive.vyMetersPerSecond),
//			drive.omegaRadiansPerSecond
//		);

		SwerveModuleState[] swerveModuleStates = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(drive, RobotState.getInstance().getGyroYaw()) : drive);
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
  public ChassisSpeeds getChassisSpeeds(boolean fieldRelative) {
    return fieldRelative ? ChassisSpeeds.fromRobotRelativeSpeeds(
      SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates()), RobotState.getInstance().getGyroYaw())
      : SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
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
		RobotState.getInstance().updateRobotPose(getModulePositions());
	}
}
