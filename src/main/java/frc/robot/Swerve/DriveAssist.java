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
import frc.robot.RobotState;

import java.util.Arrays;
import java.util.List;

public class DriveAssist {
	private final Timer _profileTimer = new Timer();
	private boolean isCurrentlyDriveAssisting = false;
  private PathPlannerTrajectory _trajectory;
	private final Field2d _currentTraj = new Field2d();

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
		double dist = RobotState.getRobotPose().getTranslation().getDistance(targetPose.getTranslation());

		if (Math.abs(angleDiff.getDegrees()) < Constants.SwerveConstants.kDriveAssistAngleThreshold && dist < Constants.SwerveConstants.kDriveAssistDistThreshold) {
			if (!isCurrentlyDriveAssisting) startingDriveAssist(targetPose, toTargetAngle, isForTags);
			isCurrentlyDriveAssisting = true;

			if (_profileTimer.get() < _trajectory.getTotalTimeSeconds())
				return calculateDriveAssist();
			else {
//				SwerveIO.getInstance().setState(SwerveDemand.SwerveState.LOCKED_AXIS);
//				SwerveIO.getInstance().updateDemand();
			}
		} else isCurrentlyDriveAssisting = false;

		return new ChassisSpeeds(
				movingDirection.getX() * Constants.SwerveConstants.maxSpeed * Constants.SwerveConstants.kSpeedFactor,
				movingDirection.getY() * Constants.SwerveConstants.maxSpeed * Constants.SwerveConstants.kSpeedFactor,
				rotation);
	}

	private ChassisSpeeds calculateDriveAssist() {
		double currentTime = this._profileTimer.get();
    PathPlannerTrajectory.State desiredState = _trajectory.sample(currentTime);
		var currentPose = RobotState.getRobotPose();

		Rotation2d heading = desiredState.heading;
		var xFeedforward = desiredState.velocityMps * heading.getCos();
		var yFeedforward = desiredState.velocityMps * heading.getSin();

		double thetaFeedback = SwerveIO.getInstance().lookAt(
			_trajectory.getEndState().targetHolonomicRotation.getDegrees(), 1);

		if (RobotState.isSimulated()) thetaFeedback = -thetaFeedback;

		Translation2d pid = SwerveIO.getInstance().pidTo(new Translation2d(desiredState.positionMeters.getX(), desiredState.positionMeters.getY()));

//		double dist = RobotState.getRobotPose().getTranslation().getDistance(_trajectory.getEndState().positionMeters);
//		double feedRatio = 0.1770 * dist - 0.008850;
//		SmartDashboard.putNumber("feedRatio", feedRatio);
//		return new ChassisSpeeds(xFeedforward * feedRatio + xFeedback * (1 - feedRatio), yFeedforward * feedRatio + yFeedback * (1 - feedRatio), thetaFeedback);
		return new ChassisSpeeds(xFeedforward + pid.getX(), yFeedforward + pid.getY(), thetaFeedback);
	}

	private void startingDriveAssist(Pose2d targetPose, Rotation2d toTargetAngle, boolean isForTags) {
    List<Translation2d> points = Arrays.asList(
				RobotState.getRobotPose().getTranslation(),
				RobotState.getRobotPose().getTranslation(),
				targetPose.getTranslation(),
				targetPose.getTranslation());

		Pathfinding.setStartPosition(RobotState.getRobotPose().getTranslation());
		Pathfinding.setGoalPosition(Constants.VisionConstants.getAmpPose().getTranslation());

		PathPlannerPath _path = new PathPlannerPath(
      points,
			Constants.SwerveConstants.AutoConstants.kConstraints,
			new GoalEndState(0, isForTags ? targetPose.getRotation().unaryMinus() : toTargetAngle));

    _trajectory = new PathPlannerTrajectory(
				_path,
				SwerveIO.getInstance().getChassisSpeeds(),
				RobotState.getRobotPose().getRotation());
		_currentTraj.getObject("Trajectory").setPoses(_path.getPathPoses());
		SmartDashboard.putData("currentTraj", _currentTraj);
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
