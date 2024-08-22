package frc.robot.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.Vision.NoteDetection;
import frc.robot.Vision.VisionIO;
import java.util.Arrays;
import java.util.List;

public abstract class SwerveIO extends SubsystemBase {
	private static SwerveIO _instance;
	private PIDController _anglePID;
	private PIDController _driveAssistXPID;
	private PIDController _driveAssistYPID;
	private boolean isDriveAssist = false;
	protected boolean isAnglePID = true;
	private boolean isBayblade = false;

	/** Returns the swerve instance, simulated/real depends on if the code is simulated/real. */
	public static SwerveIO getInstance() {
		if (_instance == null) {
			if (!RobotState.isSimulated()) _instance = new Swerve();
			else _instance = new SwerveSimulated();
		}
		return _instance;
	}

	public SwerveIO() {
		_driveAssistXPID = new PIDController(
				Constants.SwerveConstants.kDriveAssistP,
				Constants.SwerveConstants.kDriveAssistI,
				Constants.SwerveConstants.kDriveAssistD);
		_driveAssistYPID = new PIDController(
				Constants.SwerveConstants.kDriveAssistP,
				Constants.SwerveConstants.kDriveAssistI,
				Constants.SwerveConstants.kDriveAssistD);
		_anglePID = new PIDController(
				Constants.SwerveConstants.kSwerveAngleP,
				Constants.SwerveConstants.kSwerveAngleI,
				Constants.SwerveConstants.kSwerveAngleD);
		_anglePID.enableContinuousInput(
				Rotation2d.fromDegrees(-180).getDegrees(),
				Rotation2d.fromDegrees(180).getDegrees());
	}

	/**
	 * Drives the robot with the other cool stuff like drive assist and bayblade
	 *
	 * @param translation - speed percentage to move in x and y
	 * @param rotation - speed percentage to rotate the robot, positive is counterclockwise
	 */
	public void drive(Translation2d translation, double rotation) {
		ChassisSpeeds drive = new ChassisSpeeds(
				translation.getX() * Constants.SwerveConstants.kSpeedFactor * Constants.SwerveConstants.maxSpeed,
				translation.getY() * Constants.SwerveConstants.kSpeedFactor * Constants.SwerveConstants.maxSpeed,
				rotation
						* Constants.SwerveConstants.kRotationSpeedFactor
						* Constants.SwerveConstants.maxAngularVelocity);

		if (isAnglePID) {
			drive.omegaRadiansPerSecond =
					_anglePID.calculate(RobotState.getGyroYaw().getDegrees())
							* Constants.SwerveConstants.maxAngularVelocity;
		}

		if (isDriveAssist) {
			switch (RobotState.getRobotState()) {
				case NOTE_SEARCH:
					if (NoteDetection.hasTarget()) {
						Pose2d targetPose = NoteDetection.getNotePose();
						drive = calculateDriveAssist(translation, drive.omegaRadiansPerSecond, targetPose, false);
					}
					break;

				case HOLDING_NOTE:
					NetworkTableInstance.getDefault().getTable("PhotonVisionTargets").getEntry("Front").setDoubleArray(new double[]{
							VisionIO.getInstance().getClosestTag("Front").pose.toPose2d().getX(),
							VisionIO.getInstance().getClosestTag("Front").pose.toPose2d().getY(),
							VisionIO.getInstance().getClosestTag("Front").pose.toPose2d().getRotation().getDegrees()
						}
					);
					NetworkTableInstance.getDefault().getTable("PhotonVisionTargets").getEntry("BackLeft").setDoubleArray(new double[]{
							VisionIO.getInstance().getClosestTag("BackLeft").pose.toPose2d().getX(),
							VisionIO.getInstance().getClosestTag("BackLeft").pose.toPose2d().getY(),
							VisionIO.getInstance().getClosestTag("BackLeft").pose.toPose2d().getRotation().getDegrees()
						}
					);
					NetworkTableInstance.getDefault().getTable("PhotonVisionTargets").getEntry("BackRight").setDoubleArray(new double[]{
							VisionIO.getInstance().getClosestTag("BackRight").pose.toPose2d().getX(),
							VisionIO.getInstance().getClosestTag("BackRight").pose.toPose2d().getY(),
							VisionIO.getInstance().getClosestTag("BackRight").pose.toPose2d().getRotation().getDegrees()
						}
					);
					String camera = VisionIO.getInstance().getCameraByDirection(translation);
					NetworkTableInstance.getDefault().getTable("PhotonVisionTargets").getEntry("MovingDirCam").setDoubleArray(new double[]{
							VisionIO.getInstance().getClosestTag(camera).pose.toPose2d().getX(),
							VisionIO.getInstance().getClosestTag(camera).pose.toPose2d().getY(),
							VisionIO.getInstance().getClosestTag(camera).pose.toPose2d().getRotation().getDegrees()
						}
					);
					NetworkTableInstance.getDefault().getTable("PhotonVisionTargets").getEntry("MovingDirCamName").setString(camera);
					NetworkTableInstance.getDefault().getTable("PhotonVisionTargets").getEntry("MovingDir").setDoubleArray(new double[]{
							translation.getX(),
							translation.getY()
						}
					);

					if (VisionIO.getInstance().hasTargets(camera)) {
						Pose2d targetPose = VisionIO.getInstance().getClosestTag(camera).pose.toPose2d();
						drive = calculateDriveAssist(translation, drive.omegaRadiansPerSecond, targetPose, true);
					}
					break;
			}
		}

		if (isBayblade) drive.omegaRadiansPerSecond = Constants.SwerveConstants.maxAngularVelocity;

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
	 * Auto drives the robot to a given pose with an offset from the target
	 *
	 * @param targetPose - the pose and rotation to drive to
	 * @param offset - the distance from the target to stop at
	 * @return following path command
	 */
	public Command goTo(Pose2d targetPose, double offset) {
		Pose2d currentPose = RobotState.getRobotPose();

		Translation2d offsetTranslation = new Translation2d(
				offset * targetPose.getRotation().getCos(),
				offset * targetPose.getRotation().getSin());

		targetPose = new Pose2d(
				targetPose.getX() + offsetTranslation.getX(),
				targetPose.getY() + offsetTranslation.getY(),
				targetPose.getRotation());

		SmartDashboard.putNumber("Current X", currentPose.getX());
		SmartDashboard.putNumber("Current Y", currentPose.getY());
		SmartDashboard.putNumber("Current 0", currentPose.getRotation().getDegrees());

		SmartDashboard.putNumber("Auto Target X", targetPose.getX());
		SmartDashboard.putNumber("Auto Target Y", targetPose.getY());
		SmartDashboard.putNumber("Auto Target 0", targetPose.getRotation().getDegrees());

		List<Translation2d> bezierPoints = Arrays.asList(
				currentPose.getTranslation(),
				currentPose.getTranslation(),
				targetPose.getTranslation(),
				targetPose.getTranslation());

		PathPlannerPath path = new PathPlannerPath(
				bezierPoints, Constants.AutoConstants.constraints, new GoalEndState(0, targetPose.getRotation()));

		return AutoBuilder.followPath(path);
	}

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
	public void lookAt(double angle, double roundToAngle) {
		double roundedAngle = Math.round(angle / roundToAngle) * roundToAngle;
		angle = Math.abs(roundedAngle - angle) <= roundToAngle / 3 ? roundedAngle : angle;

		_anglePID.setSetpoint(angle);
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
	public void lookAt(Translation2d direction, double roundToAngle) {
		if (!(direction.getX() == 0 && direction.getY() == 0))
			lookAt(direction.getAngle().getDegrees(), roundToAngle);
	}

	/**
	 * Turns off the angle PID so the swerve rotates according to given speed in drive function.
	 * running lookAt will turn on the angle PID again
	 */
	public void turnAnglePID(boolean isAnglePID) {
		this.isAnglePID = isAnglePID;
		SmartDashboard.putBoolean("Swerve Look At", isAnglePID);
	}

	/**
	 * Calculates the drive assist
	 *
	 * @param movingDirection - the direction the robot is moving according to the driver input, field
	 *     relative
	 * @param rotation - the rotation movement of the robot according to the driver input, field
	 *     relative
	 * @param targetPose - the pose of the closest target(tags and notes), field relative
	 * @param isForTags - if the drive assist is for tags and not notes
	 * @return Calculated drive assist as chassis speeds(if the rotation is 0, use the rotation from
	 *     the drive input), field relative
	 */
	private ChassisSpeeds calculateDriveAssist(
			Translation2d movingDirection, double rotation, Pose2d targetPose, boolean isForTags) {
		if (movingDirection.getX() == 0 && movingDirection.getY() == 0) return new ChassisSpeeds(0, 0, rotation);

		Translation2d toTargetDirection =
				targetPose.getTranslation().minus(RobotState.getRobotPose().getTranslation());

		Rotation2d movingAngle = movingDirection.getAngle();
		Rotation2d toTargetAngle = toTargetDirection.getAngle();
		Rotation2d angleDiff = toTargetAngle.minus(movingAngle);

		SmartDashboard.putNumber("movingDirection X", movingDirection.getX());
		SmartDashboard.putNumber("movingDirection Y", movingDirection.getY());
		SmartDashboard.putNumber("toTargetDirection X", toTargetDirection.getX());
		SmartDashboard.putNumber("toTargetDirection Y", toTargetDirection.getY());
		SmartDashboard.putNumber("angleDiff", angleDiff.getDegrees());

		if (Math.abs(angleDiff.getDegrees()) < Constants.SwerveConstants.kDriveAssistThreshold) {
			double anglePIDMeasurement = RobotState.getRobotPose().getRotation().getDegrees();
			double anglePIDSetpoint = isForTags
					? targetPose
							.getRotation()
							.minus(Rotation2d.fromDegrees(180))
							.getDegrees()
					: toTargetAngle.getDegrees();

			ChassisSpeeds driveAssist = new ChassisSpeeds(
					_driveAssistXPID.calculate(RobotState.getRobotPose().getX(), targetPose.getX())
							* Constants.SwerveConstants.maxSpeed,
					_driveAssistYPID.calculate(RobotState.getRobotPose().getY(), targetPose.getY())
							* Constants.SwerveConstants.maxSpeed,
					_anglePID.calculate(anglePIDMeasurement, anglePIDSetpoint)
							* Constants.SwerveConstants.maxAngularVelocity);

			SmartDashboard.putNumber(
					"Drive Assist X Error",
					targetPose.getX() - RobotState.getRobotPose().getX());
			SmartDashboard.putNumber(
					"Drive Assist Y Error",
					targetPose.getY() - RobotState.getRobotPose().getY());
			return driveAssist;
		}

		return new ChassisSpeeds(movingDirection.getX(), movingDirection.getY(), rotation);
	}

	/**
	 * Sets whether the drive assist is on
	 *
	 * @param isDriveAssist
	 */
	public void setIsDriveAssist(boolean isDriveAssist) {
		this.isDriveAssist = isDriveAssist;
		SmartDashboard.putBoolean("Swerve Drive Assist", isDriveAssist);
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

	/** Logs info about the modules and swerve */
	public void log() {
		// TODO: make this work
	}

	@Override
	public void periodic() {
		log();
	}
}
