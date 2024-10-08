package frc.robot.NinjasLib.Swerve.PathFollowing;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.NinjasLib.Swerve.SwerveIO;
import frc.robot.RobotState;
import java.util.List;

public class PathFollower {
	private Timer _profileTimer;

	private boolean started = false;

	private PathPlannerTrajectory _trajectory;
	private Field2d _trajectoryLog = new Field2d();

	public PathFollower() {
		_profileTimer = new Timer();
	}

	/**
	 * follows path to given target
	 * @param targetPose - given target
	 * @return Calculated chassis speeds, field relative
	 */
	public ChassisSpeeds followPath(Pose2d targetPose) {
		if (!started) {
			starting(targetPose);
			started = true;
		}

		return calculateFollowing();
	}

	private ChassisSpeeds calculateFollowing() {
		PathPlannerTrajectory.State desiredState = _trajectory.sample(_profileTimer.get());

		Rotation2d heading = desiredState.heading;
		double xFeedforward = desiredState.velocityMps * heading.getCos();
		double yFeedforward = desiredState.velocityMps * heading.getSin();

		double thetaFeedback = SwerveIO.getInstance().lookAt(desiredState.targetHolonomicRotation.getDegrees(), 1);
		Translation2d feedback = SwerveIO.getInstance()
				.pidTo(new Translation2d(desiredState.positionMeters.getX(), desiredState.positionMeters.getY()));

		return new ChassisSpeeds(xFeedforward + feedback.getX(), yFeedforward + feedback.getY(), thetaFeedback);
	}

	private void starting(Pose2d targetPose) {
		List<Translation2d> points = List.of(
				RobotState.getRobotPose().getTranslation(),
				RobotState.getRobotPose().getTranslation(),
				targetPose.getTranslation(),
				targetPose.getTranslation());

		PathPlannerPath _path = new PathPlannerPath(
				points,
				Constants.SwerveConstants.AutoConstants.kConstraints,
				new GoalEndState(
						0,
						RobotState.isSimulated() ? targetPose.getRotation().unaryMinus() : targetPose.getRotation()));

		_trajectory = new PathPlannerTrajectory(
				_path,
				SwerveIO.getInstance().getChassisSpeeds(),
				RobotState.getRobotPose().getRotation());

		_trajectoryLog.getObject("Trajectory").setPoses(_path.getPathPoses());
		SmartDashboard.putData("PathFollower Trajectory", _trajectoryLog);

		_profileTimer.restart();
	}

	/**
	 * Call this when turning off path follower.
	 * You need to call this when turning off path follower for some logic going on in here. DON'T ASK!
	 */
	public void stop() {
		started = false;
		_trajectory = null;
	}

	/**
	 * @return Whether the path following was finished, will return false if not started
	 */
	public boolean isFinished() {
		return _trajectory != null && _profileTimer.get() >= _trajectory.getTotalTimeSeconds();
	}
}
