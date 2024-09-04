package frc.robot.Swerve;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.Arrays;
import java.util.List;

public class DriveAssist {
	private PIDController translationPID;

	private Timer _profileTimer;

	private boolean isCurrentlyDriveAssisting = false;

	private PIDController rotationPID;
	private PathPlannerTrajectory trajectory;
	private static StructPublisher<Pose2d> _robotPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Wanted Pose", Pose2d.struct)
			.publish();

	private Field2d currentTraj = new Field2d();

	public DriveAssist(PIDController anglePID) {

		_profileTimer = new Timer();

		translationPID = new PIDController(
				Constants.SwerveConstants.kDriveAssistP,
				Constants.SwerveConstants.kDriveAssistI,
				Constants.SwerveConstants.kDriveAssistD);
		rotationPID = anglePID;
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
	public ChassisSpeeds driveAssist(
			Translation2d movingDirection, double rotation, Pose2d targetPose, boolean isForTags) {
		if (movingDirection.getX() == 0 && movingDirection.getY() == 0) return new ChassisSpeeds(0, 0, rotation);

		Translation2d toTargetDirection =
				targetPose.getTranslation().minus(RobotState.getRobotPose().getTranslation());

		Rotation2d movingAngle = movingDirection.getAngle();
		Rotation2d toTargetAngle = toTargetDirection.getAngle();
		Rotation2d angleDiff = toTargetAngle.minus(movingAngle);

		if (Math.abs(angleDiff.getDegrees()) < Constants.SwerveConstants.kDriveAssistThreshold) {
			if (!isCurrentlyDriveAssisting) startingDriveAssist(targetPose, toTargetAngle, isForTags);
			isCurrentlyDriveAssisting = true;

			return calculateDriveAssist();
		} else isCurrentlyDriveAssisting = false;

		return new ChassisSpeeds(
				movingDirection.getX() * Constants.SwerveConstants.maxSpeed * Constants.SwerveConstants.kSpeedFactor,
				movingDirection.getY() * Constants.SwerveConstants.maxSpeed * Constants.SwerveConstants.kSpeedFactor,
				rotation);
	}

	private ChassisSpeeds calculateDriveAssist() {
		double currentTime = this._profileTimer.get();
		// Determine desired state based on where the robot should be at the current time in the path
		PathPlannerTrajectory.State desiredState = trajectory.sample(currentTime);
		var currentPose = RobotState.getRobotPose();

		Rotation2d heading = desiredState.heading;
		// Calculate our target velocity based on current pose and desired state
		var vx = desiredState.velocityMps * Math.cos(heading.getRadians());
		var vy = desiredState.velocityMps * Math.sin(heading.getRadians());
		_robotPosePublisher.accept(desiredState.getTargetHolonomicPose());

		double desiredThetaSpeeds = rotationPID.calculate(
				currentPose.getRotation().getDegrees(), desiredState.targetHolonomicRotation.getDegrees());
		double xFeedback = translationPID.calculate(
				currentPose.getX(), desiredState.getTargetHolonomicPose().getX());
		double yFeedback = translationPID.calculate(
				currentPose.getY(), desiredState.getTargetHolonomicPose().getY());

		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(vx + xFeedback, vy + yFeedback, desiredThetaSpeeds);
		SmartDashboard.putNumber("vX speeds", vx);
		SmartDashboard.putNumber("vY speeds", vy);
		SmartDashboard.putNumber("vTheta speeds", desiredThetaSpeeds);
		return chassisSpeeds;
	}

	private void startingDriveAssist(Pose2d targetPose, Rotation2d toTargetAngle, boolean isForTags) {
		List<Translation2d> _points = Arrays.asList(
				RobotState.getRobotPose().getTranslation(),
				RobotState.getRobotPose().getTranslation(),
				targetPose.getTranslation(),
				targetPose.getTranslation());

		Pathfinding.setStartPosition(RobotState.getRobotPose().getTranslation());
		Pathfinding.setGoalPosition(Constants.VisionConstants.getAmpPose().getTranslation());

		PathPlannerPath _path = new PathPlannerPath(
				_points,
				Constants.AutoConstants.constraints,
				new GoalEndState(1, isForTags ? targetPose.getRotation().unaryMinus() : toTargetAngle));

		currentTraj.getObject("Trajectory").setPoses(_path.getPathPoses());
		SmartDashboard.putData("traj", currentTraj);

		trajectory = new PathPlannerTrajectory(
				_path,
				SwerveIO.getInstance().getChassisSpeeds(),
				RobotState.getRobotPose().getRotation());

		_profileTimer.restart();
	}

	/**
	 * Call me when turning off drive assist.
	 * You need to call me when turning off drive assist for some logic going on in here. DON'T ASK!
	 */
	public void turnOffDriveAssist() {
		isCurrentlyDriveAssisting = false;
	}
}
