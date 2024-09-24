package frc.robot.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AbstractClasses.StateManager;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import frc.robot.Swerve.SwerveDemand.SwerveState;

public abstract class SwerveIO extends StateManager {
	private static SwerveIO _instance;

	private PIDController _anglePID;
	private PIDController _axisPID;
	private DriveAssist _driveAssist;

	private SwerveDemand _demand;
	private SwerveState _state;
	private SwerveState _previousState;

	/** Returns the swerve instance, simulated/real depends on if the code is simulated/real. */
	public static SwerveIO getInstance() {
		if (_instance == null) {
			if (!RobotState.isSimulated()) _instance = new Swerve();
			else _instance = new SwerveSimulated();
		}
		return _instance;
	}

	public SwerveIO() {
		_state = SwerveState.DEFAULT;
		_previousState = SwerveState.DEFAULT;
		_demand = new SwerveDemand();

		_anglePID = new PIDController(Constants.AutoConstants.kPTheta, 0, 0);

		_anglePID.enableContinuousInput(
				Rotation2d.fromDegrees(-180).getDegrees(),
				Rotation2d.fromDegrees(180).getDegrees());

		_axisPID = new PIDController(
				SwerveConstants.kSwerveAxisLockP, SwerveConstants.kSwerveAxisLockI, SwerveConstants.kSwerveAxisLockD);

		_driveAssist = new DriveAssist();
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
				driverInput.omegaRadiansPerSecond
						* SwerveConstants.kRotationSpeedFactor
						* SwerveConstants.maxAngularVelocity);
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
		return _anglePID.calculate(RobotState.getGyroYaw().getDegrees(), angle);
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
	 * @return Whether drive assist is enabled
	 */
	public boolean isFinishedDriveAssist() {
		return _driveAssist.isFinished();
	}

	/** Logs info about the modules and swerve */
	private void log() {
		// TODO: make this work
	}
	/**
	 * Makes the swerve be locked to an axis with a pid that ensures that. The driver input let the driver move along the axis.
	 *
	 * @param angle          angle of the axis
	 * @param phase          how much the axis is moved from the origin of the field in meters
	 * @param driverInput    the driver controller input
	 * @param isXDriverInput whether to let the driver drive along the axis by the x of the joystick input or the y
	 */
	private void lockAxis(Rotation2d angle, double phase, ChassisSpeeds driverInput, boolean isXDriverInput) {
		Translation2d axis = new Translation2d(1, angle);
		Translation2d perpendicularAxis = axis.rotateBy(Rotation2d.fromDegrees(90));
		Translation2d robotPose = RobotState.getRobotPose().getTranslation();

		double a = -axis.getY();
		double b = axis.getX();
		double c = -phase * Math.sqrt(a * a + b * b);
		double error = -(a * robotPose.getX() + b * robotPose.getY() + c) / Math.sqrt(a * a + b * b);
		Translation2d pid = perpendicularAxis.times(_axisPID.calculate(-error));

		Translation2d driver =
				axis.times(isXDriverInput ? -driverInput.vyMetersPerSecond : driverInput.vxMetersPerSecond);

		ChassisSpeeds speeds = new ChassisSpeeds(
				driver.getX() + pid.getX(), driver.getY() + pid.getY(), driverInput.omegaRadiansPerSecond);
		drive(speeds);
	}

	/**
	 * Set the current state of the swerve so it will work according
	 * @param state the wanted state
	 */
	public void setState(SwerveState state) {

		_state = state;

		SmartDashboard.putString("Swerve State", _state.toString());
	}

	/**
	 * @return the current state of the swerve
	 */
	public SwerveState getState() {
		return _state;
	}

	/**
	 * @return the previous state of the swerve, the state it was before changing it
	 */
	public SwerveState getPreviousState() {
		return _previousState;
	}

	/**
	 * Update the swerve demand. The demand of the swerve is the values the swerve should work with when working according to states.
	 * In this case it is the driver input to drive according to.
	 * @param driverInput the controller input of the driver
	 * @see #updateDemand(ChassisSpeeds)
	 * @see #updateDemand(Pose2d)
	 * @see #updateDemand(Rotation2d, double, boolean isXDriverInput)
	 * @see #updateDemand(ChassisSpeeds, boolean)
	 * @see #updateDemand(Translation2d)
	 */
	public void updateDemand(ChassisSpeeds driverInput) {
		_demand.driverInput = driverInput;
	}

	/**
	 * Update the swerve demand. The demand of the swerve is the values the swerve should work with when working according to states.
	 * In this case it is the velocity to make the swerve drive according to.
	 * @param velocity the wanted velocity
	 * @param fieldRelative whether to drive field relatively
	 * @see #updateDemand(ChassisSpeeds)
	 * @see #updateDemand(Pose2d)
	 * @see #updateDemand(Rotation2d, double, boolean isXDriverInput)
	 * @see #updateDemand(ChassisSpeeds, boolean)
	 * @see #updateDemand(Translation2d)
	 */
	public void updateDemand(ChassisSpeeds velocity, boolean fieldRelative) {
		_demand.velocity = velocity;
		_demand.fieldRelative = fieldRelative;
	}

	/**
	 * Update the swerve demand. The demand of the swerve is the values the swerve should work with when working according to states.
	 * In this case it is a target pose that could be used in many states.
	 * @param targetPose the wanted pose
	 * @see #updateDemand(ChassisSpeeds)
	 * @see #updateDemand(Pose2d)
	 * @see #updateDemand(Rotation2d, double, boolean isXDriverInput)
	 * @see #updateDemand(ChassisSpeeds, boolean)
	 * @see #updateDemand(Translation2d)
	 */
	public void updateDemand(Pose2d targetPose) {
		setState(SwerveState.POSITION);
		_demand.targetPose = targetPose;
	}

	/**
	 * Update the swerve demand. The demand of the swerve is the values the swerve should work with when working according to states.
	 * In this case it is the angle and phase to make an axis for the swerve to be locked to.
	 *
	 * @param angle          the angle of the axis
	 * @param phase          the distance between the axis and the field origin in meters
	 * @param isXDriverInput whether to let the driver drive along the axis by the x of the joystick input or the y
	 * @see #updateDemand(ChassisSpeeds)
	 * @see #updateDemand(Pose2d)
	 * @see #updateDemand(Rotation2d, double, boolean isXDriverInput)
	 * @see #updateDemand(ChassisSpeeds, boolean)
	 * @see #updateDemand(Translation2d)
	 */
	public void updateDemand(Rotation2d angle, double phase, boolean isXDriverInput) {
		setState(SwerveState.LOCKED_AXIS);
		_demand.angle = angle;
		_demand.phase = phase;
		_demand.isXDriverInput = isXDriverInput;
	}

	/**
	 * Update the swerve demand. The demand of the swerve is the values the swerve should work with when working according to states.
	 * In this case it is the translation to make the swerve look at.
	 * @param lookAtTranslation the wanted looking direction
	 * @see #updateDemand(ChassisSpeeds)
	 * @see #updateDemand(Pose2d)
	 * @see #updateDemand(Rotation2d, double, boolean isXDriverInput)
	 * @see #updateDemand(ChassisSpeeds, boolean)
	 * @see #updateDemand(Translation2d)
	 */
	public void updateDemand(Translation2d lookAtTranslation) {
		_demand.lookAtTranslation = lookAtTranslation;
	}

	public void followPath(Pose2d targetPose2d) {
		setState(SwerveState.FOLLOW_PATH);
	}

	@Override
	public void periodic() {
		switch (_state) {
			case DEFAULT:
				drive(_demand.driverInput);
				break;
			case FOLLOW_PATH:
				_driveAssist.driveAssist(_demand.targetPose, new Rotation2d());
				drive(_driveAssist.getCurrentDemand(), true);
				break;
			case BAYBLADE:
				drive(new ChassisSpeeds(
						_demand.driverInput.vxMetersPerSecond,
						_demand.driverInput.vyMetersPerSecond,
						SwerveConstants.maxAngularVelocity));
				break;

			case LOOK_AT_ANGLE:
				drive(new ChassisSpeeds(
						_demand.driverInput.vxMetersPerSecond,
						_demand.driverInput.vyMetersPerSecond,
						lookAt(_demand.lookAtTranslation, 45)));
				break;

			case LOOK_AT_TARGET:
				Translation2d lookAtTranslation = _demand.targetPose
						.getTranslation()
						.minus(RobotState.getRobotPose().getTranslation());
				lookAtTranslation = RobotState.isSimulated()
						? new Translation2d(lookAtTranslation.getX(), -lookAtTranslation.getY())
						: lookAtTranslation;
				drive(new ChassisSpeeds(
						_demand.driverInput.vxMetersPerSecond,
						_demand.driverInput.vyMetersPerSecond,
						lookAt(lookAtTranslation, 45)));
				break;

			case POSITION:
				lockPosition(_demand.targetPose);
				break;

			case VELOCITY:
				drive(_demand.velocity, _demand.fieldRelative);
				break;

			case LOCKED_AXIS:
				lockAxis(_demand.angle, _demand.phase, _demand.driverInput, _demand.isXDriverInput);

			default:
				break;
		}

		log();
		super.periodic();
		_previousState = _state;
	}

	private void lockPosition(Pose2d targetPose) {
		drive(
				new ChassisSpeeds(
						_axisPID.calculate(
								targetPose.getX(), RobotState.getRobotPose().getX()),
						_axisPID.calculate(
								targetPose.getY(), RobotState.getRobotPose().getY()),
						_demand.driverInput.omegaRadiansPerSecond),
				true);
	}

	private void goToPose(Pose2d targetPose) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'goToPose'");
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToPeriodicMap(() -> setState(SwerveState.DEFAULT), RobotStates.IDLE);
		addFunctionToOnChangeMap(() -> _driveAssist.resetPath(), RobotStates.PREPARE_AMP_OUTAKE);
		addFunctionToPeriodicMap(
				() -> {
					_demand.targetPose = VisionConstants.getOffsetTagPose(VisionConstants.getTagPose(6), 1.25);
					double dist =
							RobotState.getRobotPose().getTranslation().getDistance(_demand.targetPose.getTranslation());
					if (dist < 2) {
						SmartDashboard.putNumber("dista", dist);
						followPath(null);
					}
				},
				RobotStates.PREPARE_AMP_OUTAKE);
		addFunctionToPeriodicMap(
			() -> updateDemand(Rotation2d.fromDegrees(90), -1.7, true), RobotStates.AMP_OUTAKE_READY);
	}
}
