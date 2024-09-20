package frc.robot.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;
import frc.robot.Swerve.SwerveDemand.SwerveState;
import frc.robot.Vision.NoteDetection;

public abstract class SwerveIO extends SubsystemBase {
	private static SwerveIO _instance;

	private final PIDController _anglePID;
	private PIDController _xPID;
	private PIDController _yPID;
	private final PIDController _axisPID;
	private final DriveAssist _driveAssist;

	private boolean isDriveAssist = false;
	private final boolean isBayblade = false;

	private final SwerveDemand _demand;
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

		_anglePID = new PIDController(
			SwerveConstants.AutoConstants.kPTheta, SwerveConstants.AutoConstants.kITheta, SwerveConstants.AutoConstants.kDTheta);

		_anglePID.enableContinuousInput(
				Rotation2d.fromDegrees(-180).getDegrees(),
				Rotation2d.fromDegrees(180).getDegrees());

		_axisPID = new PIDController(
			SwerveConstants.AutoConstants.kP, SwerveConstants.AutoConstants.kI, SwerveConstants.AutoConstants.kD);

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

		if (isDriveAssist) {
			switch (RobotState.getRobotState()) {
				case NOTE_SEARCH:
					if (NoteDetection.hasTarget()) {
						Pose2d targetPose = NoteDetection.getNotePose();
						drive = _driveAssist.driveAssist(
								new Translation2d(driverInput.vxMetersPerSecond, driverInput.vyMetersPerSecond),
								drive.omegaRadiansPerSecond,
								targetPose,
								false);
					}
					break;

				case HOLDING_NOTE:
					Pose2d targetPose = VisionConstants.getOffsetTagPose(VisionConstants.getTagByDirection(new Translation2d(drive.vxMetersPerSecond, drive.vyMetersPerSecond)).pose.toPose2d(), 1);
//					Pose2d targetPose = VisionConstants.getOffsetTagPose(VisionConstants.getTagPose(6), 1.25);
					NetworkTableInstance.getDefault()
							.getTable("Assist Target Offset")
							.getEntry("pose")
							.setDoubleArray(new double[] {targetPose.getX(), targetPose.getY()});

					drive = _driveAssist.driveAssist(
							new Translation2d(driverInput.vxMetersPerSecond, driverInput.vyMetersPerSecond),
							drive.omegaRadiansPerSecond,
							targetPose,
							true);
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

	public Translation2d pidTo(Translation2d target) {
		return new Translation2d(_xPID.calculate(RobotState.getRobotPose().getX(), target.getX()),
			_yPID.calculate(RobotState.getRobotPose().getY(), target.getY()));
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

	/** Logs info about the modules and swerve */
	private void log() {
		// TODO: make this work
	}

	private void goToPose(Pose2d pose, ChassisSpeeds driverInput) {
		Pathfinding.setGoalPosition(pose.getTranslation());
		Pathfinding.setStartPosition(RobotState.getRobotPose().getTranslation());
		PathPlannerPath path = Pathfinding.getCurrentPath(SwerveConstants.AutoConstants.kConstraints, new GoalEndState(0, pose.getRotation()));
		PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, getChassisSpeeds(), RobotState.getRobotPose().getRotation());

		Translation2d pid = pidTo(trajectory.getState(2).positionMeters);
		drive(new ChassisSpeeds(pid.getX() + driverInput.vxMetersPerSecond, pid.getY() + driverInput.vyMetersPerSecond, +driverInput.omegaRadiansPerSecond), true);

		Field2d trajectoryLog = new Field2d();
		trajectoryLog.getObject("Trajectory").setPoses(path.getPathPoses());
		SmartDashboard.putData("Pathfinding Trajectory", trajectoryLog);
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

		Translation2d driver = axis.times(isXDriverInput ? -driverInput.vyMetersPerSecond : driverInput.vxMetersPerSecond);

		ChassisSpeeds speeds = new ChassisSpeeds(
				driver.getX() + pid.getX(), driver.getY() + pid.getY(), driverInput.omegaRadiansPerSecond);
		drive(speeds);
	}

	/**
	 * Set the current state of the swerve so it will work according
	 * @param state the wanted state
	 */
	public void setState(SwerveState state) {
		_previousState = _state;
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

	@Override
	public void periodic() {
		switch (_state) {
			case DEFAULT:
				drive(_demand.driverInput);
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
				goToPose(_demand.targetPose, _demand.driverInput);
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
	}
}
