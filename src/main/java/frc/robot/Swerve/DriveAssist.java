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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.RobotState;

import java.util.Arrays;
import java.util.List;

public class DriveAssist {
	private PIDController xPID;
	private PIDController yPID;
	private Timer _profileTimer;
	private boolean isCurrentlyDriveAssisting = false;
	private PathPlannerTrajectory _trajectory;
	private Field2d _currentTraj = new Field2d();
	private ChassisSpeeds _currentDemand;

	public DriveAssist() {
		_profileTimer = new Timer();

		xPID = new PIDController(
				Constants.SwerveConstants.kDriveAssistP,
				Constants.SwerveConstants.kDriveAssistI,
				Constants.SwerveConstants.kDriveAssistD);

		yPID = new PIDController(
				Constants.SwerveConstants.kDriveAssistP,
				Constants.SwerveConstants.kDriveAssistI,
				Constants.SwerveConstants.kDriveAssistD);
		_currentDemand = new ChassisSpeeds();
	}

	/**
	 * Calculates the drive assist
	 * @param targetPose
	 *
	 * @param movingDirection - the direction the robot is moving according to the driver input, field
	 *     relative
	 * @param rotation - the rotation movement of the robot according to the driver input, field
	 *     relative
	 * @param targetPose - the pose of the closest target(tags and notes), field relative
	 * @param toTargetAngle
	 * @param isForTags - if the drive assist is for tags and not notes
	 * @return Calculated drive assist as chassis speeds(if the rotation is 0, use the rotation from
	 *     the drive input), field relative
	 */
	public void driveAssist(Pose2d targetPose, Rotation2d toTargetAngle) {
		if (!isCurrentlyDriveAssisting) {
			startingDriveAssist(targetPose, toTargetAngle);
			isCurrentlyDriveAssisting = true;
		}
		if (!isFinished()) {

			_currentDemand = calculateDriveAssist();

		} else {
			_profileTimer.stop();
		}
	}

	private ChassisSpeeds calculateDriveAssist() {
		double currentTime = this._profileTimer.get();
		PathPlannerTrajectory.State desiredState = _trajectory.sample(currentTime);
		var currentPose = RobotState.getRobotPose();

		Rotation2d heading = desiredState.heading;
		var xFeedforward = desiredState.velocityMps * heading.getCos();
		var yFeedforward = desiredState.velocityMps * heading.getSin();


		double thetaFeedback = SwerveIO.getInstance()
			.lookAt(desiredState.targetHolonomicRotation.getDegrees(), 1);

		double xFeedback = xPID.calculate(currentPose.getX(), desiredState.positionMeters.getX());
		double yFeedback = yPID.calculate(currentPose.getY(), desiredState.positionMeters.getY());

		return new ChassisSpeeds(xFeedforward + xFeedback,
			yFeedforward + yFeedback,
			thetaFeedback);
	}

	private void startingDriveAssist(Pose2d targetPose, Rotation2d toTargetAngle) {
		List<Translation2d> points = Arrays.asList(
				RobotState.getRobotPose().getTranslation(),
				RobotState.getRobotPose().getTranslation(),
				targetPose.getTranslation(),
				targetPose.getTranslation());

		Pathfinding.setStartPosition(RobotState.getRobotPose().getTranslation());
		Pathfinding.setGoalPosition(VisionConstants.getTagPose(1).getTranslation());

		PathPlannerPath _path = new PathPlannerPath(
				points,
				Constants.AutoConstants.kConstraints,
				new GoalEndState(0, targetPose.getRotation().unaryMinus()));

		_trajectory = new PathPlannerTrajectory(
				_path,
				SwerveIO.getInstance().getChassisSpeeds(),
				RobotState.getRobotPose().getRotation());
		_currentTraj.getObject("Trajectory").setPoses(_path.getPathPoses());
		SmartDashboard.putData("currentTraj", _currentTraj);
		_profileTimer.restart();
	}

	public void resetPath() {
		_profileTimer.reset();
		isCurrentlyDriveAssisting = false;
	}
	/**
	 * Call me when turning off drive assist.
	 * You need to call me when turning off drive assist for some logic going on in here. DON'T ASK!
	 */
	public void turnOffDriveAssist() {
		isCurrentlyDriveAssisting = false;
	}

	public boolean isFinished() {
		return _trajectory != null && _profileTimer.get() >= _trajectory.getTotalTimeSeconds();
	}

	public ChassisSpeeds getCurrentDemand() {
		return _currentDemand;
	}
}
