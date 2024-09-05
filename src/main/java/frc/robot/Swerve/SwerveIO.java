package frc.robot.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.Vision.NoteDetection;

public abstract class SwerveIO extends SubsystemBase {
	private static SwerveIO _instance;

	private PIDController _anglePID;
	private PIDController _axisPID;
	private DriveAssist _driveAssist;

	private boolean isDriveAssist = false;
	private boolean isAnglePID = true;
	private boolean isBayblade = false;

	private SwerveDemand _demand;

	/** Returns the swerve instance, simulated/real depends on if the code is simulated/real. */
	public static SwerveIO getInstance() {
		if (_instance == null) {
			if (!RobotState.isSimulated()) _instance = new Swerve();
			else _instance = new SwerveSimulated();
		}
		return _instance;
	}

	public SwerveIO() {
		_anglePID = new PIDController(
				SwerveConstants.kSwerveAngleP, SwerveConstants.kSwerveAngleI, SwerveConstants.kSwerveAngleD);

		_anglePID.enableContinuousInput(
				Rotation2d.fromDegrees(-180).getDegrees(),
				Rotation2d.fromDegrees(180).getDegrees());

		_axisPID = new PIDController(SwerveConstants.kSwerveAxisLockP, SwerveConstants.kSwerveAxisLockI, SwerveConstants.kSwerveAxisLockD);

		_driveAssist = new DriveAssist(_anglePID);
	}

	/**
	 * Drives the robot with the other cool stuff like drive assist and bayblade
	 *
	 * @param driverInput - speed percentage to move in x and y, and rotate the robot, positive is forward, left, counterclockwise
	 */
	private void drive(ChassisSpeeds driverInput) {
		ChassisSpeeds drive = new ChassisSpeeds(
			driverInput.vxMetersPerSecond * SwerveConstants.kSpeedFactor * SwerveConstants.maxSpeed,
			driverInput.vyMetersPerSecond * SwerveConstants.kSpeedFactor * SwerveConstants.maxSpeed,
			driverInput.omegaRadiansPerSecond * SwerveConstants.kRotationSpeedFactor * SwerveConstants.maxAngularVelocity);

		if (isDriveAssist) {
			switch (RobotState.getRobotState()) {
				case NOTE_SEARCH:
					if (NoteDetection.hasTarget()) {
						Pose2d targetPose = NoteDetection.getNotePose();
						drive = _driveAssist.driveAssist(new Translation2d(driverInput.vxMetersPerSecond, driverInput.vyMetersPerSecond), drive.omegaRadiansPerSecond, targetPose, false);
					}
					break;

				case HOLDING_NOTE:
					//					Pose2d targetPose = VisionConstants.getFieldLayout().getTagPose(6).get().toPose2d();
					//					Pose2d targetPose =
					// VisionConstants.getOffsetTagPose(VisionConstants.getTagByDirection(translation).pose.toPose2d(),
					// 0.5);
					Pose2d targetPose = VisionConstants.getOffsetTagPose(VisionConstants.getTagPose(6), 1.5);
					NetworkTableInstance.getDefault()
							.getTable("Assist Target Offset")
							.getEntry("pose")
							.setDoubleArray(new double[] {targetPose.getX(), targetPose.getY()});

					drive = _driveAssist.driveAssist(new Translation2d(driverInput.vxMetersPerSecond, driverInput.vyMetersPerSecond), drive.omegaRadiansPerSecond, targetPose, true);
					break;
			}
		}

		if (isBayblade) drive.omegaRadiansPerSecond = SwerveConstants.maxAngularVelocity;

		drive(drive, SwerveConstants.kFieldRelative);
	}

	/**
	 * Drives the robot
	 *
	 * @param drive - chassis speeds to drive according to
	 * @param fieldRelative - Whether to move to robot relative to the field or the robot
	 */
	public abstract void drive(ChassisSpeeds drive, boolean fieldRelative);

	/**
	 * @return drive encoder value of each module, angle of each module
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];
		for (int i = 0; i < positions.length; i++) positions[i] = new SwerveModulePosition();
		return positions;
	}

	public abstract ChassisSpeeds getChassisSpeeds();

	/**
	 * Makes the swerve use PID to look at the given angle
	 *
	 * @param angle - the angle to look at
	 * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round
	 *     the given angle to the nearest 0, 45, 90, 135... it rounds the angle only if the rounded
	 *     angle is close enough to the given angle so for example if the given angle is 28 and the
	 *     rounded angle is 45 it won't round. if you write 1 as the roundToAngle there will be no
	 *     rounding, DON'T USE 0 (division by zero error)
	 */
	public double lookAt(double angle, double roundToAngle) {
		double roundedAngle = Math.round(angle / roundToAngle) * roundToAngle;
		angle = Math.abs(roundedAngle - angle) <= roundToAngle / 3 ? roundedAngle : angle;

		return _anglePID.calculate(RobotState.getGyroYaw().getDegrees(), angle) * SwerveConstants.maxAngularVelocity;
	}

	/**
	 * Makes the swerve use PID to look according to the given direction
	 *
	 * @param direction - the direction vector to look
	 * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round
	 *     the given angle (calculated from direction) to the nearest 0, 45, 90, 135... it rounds the
	 *     angle only if the rounded angle is close enough to the given angle so for example if the
	 *     given angle is 28 and the rounded angle is 45 it won't round. if you write 1 as the
	 *     roundToAngle there will be no rounding, DON'T USE 0 (division by zero error)
	 */
	public double lookAt(Translation2d direction, double roundToAngle) {
		if (!(direction.getX() == 0 && direction.getY() == 0))
			return lookAt(direction.getAngle().getDegrees(), roundToAngle);

		return 0;
	}

	/**
	 * Turns off the angle PID so the swerve rotates according to given speed in drive function.
	 * running lookAt will turn on the angle PID again
	 */
	public void setAnglePID(boolean isAnglePID) {
		this.isAnglePID = isAnglePID;
		SmartDashboard.putBoolean("Swerve Look At", isAnglePID);
	}

	/**
	 * @return Whether angle pid is enabled (look at mode)
	 */
	public boolean isAnglePID() {
		return isAnglePID;
	}

	/**
	 * Enable/Disable drive assist
	 */
	public void setDriveAssist(boolean isDriveAssist) {
		this.isDriveAssist = isDriveAssist;

		if (!isDriveAssist) _driveAssist.turnOffDriveAssist();

		SmartDashboard.putBoolean("Swerve Drive Assist", isDriveAssist);
	}

	/**
	 * @return Whether drive assist is enabled
	 */
	public boolean isDriveAssist() {
		return isDriveAssist;
	}

	/**
	 * Sets whether the bayblade mode is on, if it is on the swerve will rotate full speed nonstop
	 *
	 * @param isBayblade - true if bayblade mode should be on
	 */
	public void setBaybladeMode(boolean isBayblade) {
		this.isBayblade = isBayblade;
		SmartDashboard.putBoolean("Swerve Bayblade", isBayblade);
	}

	/**
	 * @return Whether the swerve is in bayblade mode
	 */
	public boolean isBaybladeMode() {
		return isBayblade;
	}

	/** Logs info about the modules and swerve */
	private void log() {
		// TODO: make this work
	}

	private void goToPose(Pose2d pose) {
		// TODO: make this work
	}

	private void lockAxis(Translation2d axis, double phase, ChassisSpeeds driverInput) {
		Translation2d perpendicularAxis = axis.rotateBy(Rotation2d.fromDegrees(90));

		Translation2d robotPose = RobotState.getRobotPose().getTranslation();

		double a = -axis.getY();
		double b = axis.getX();
		double c = -phase * Math.sqrt(a * a + b * b);
		double error = Math.abs(a * robotPose.getX() + b * robotPose.getY() + c) / Math.sqrt(a * a + b * b);
		Translation2d pid = perpendicularAxis.times(_axisPID.calculate(-error));

		Translation2d driver = axis.times(driverInput.vxMetersPerSecond);

		ChassisSpeeds speeds = new ChassisSpeeds(driver.getX() + pid.getX(), driver.getY() + pid.getY(), driverInput.omegaRadiansPerSecond);
		drive(speeds);
	}

	public void set(SwerveDemand demand) {
		_demand = demand;
	}

	@Override
	public void periodic() {
		if(_demand != null)
			switch (_demand.state) {
				case DRIVER:
					drive(_demand.chassisSpeeds);
					break;

				case POSITION:
					goToPose(_demand.targetPose);
					break;

				case VELOCITY:
					drive(_demand.chassisSpeeds, _demand.fieldRelative);
					break;

				case LOCKED_AXIS:
					lockAxis(_demand.axis, _demand.phase, _demand.chassisSpeeds);

				default:
					break;
			}
		log();
	}
}
