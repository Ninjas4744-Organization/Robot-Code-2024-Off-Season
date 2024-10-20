package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.NinjasLib.DataClasses.VisionEstimation;
import frc.robot.NinjasLib.Swerve.Swerve;
import frc.robot.NinjasLib.Swerve.SwerveIO;

public class RobotState {
	public enum RobotStates {
		IDLE,
		INTAKE,
		INDEX,
		INDEX_BACK,
		CLOSE,
		RESET,
		SHOOT_SPEAKER_PREPARE,
		SHOOT_AMP_PREPARE,
		SHOOT_SPEAKER_READY,
		SHOOT_AMP_READY,
		SHOOT,
		NOTE_SEARCH,
		NOTE_IN_INDEXER,
		CLIMB_PREPARE,
		CLIMB_READY,
		CLIMB,
		CLIMBED,
		DRIVE_TO_AMP,
		DRIVE_TO_SOURCE,
		TESTING,
		OUTTAKE,
		DELIVERY
	}

	private static RobotStates robotState = RobotStates.IDLE;
	private static AHRS navX = new AHRS();
	private static SwerveDrivePoseEstimator poseEstimator;
	private static StructPublisher<Pose2d> _robotPosePublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Robot Pose", Pose2d.struct)
			.publish();

	private static DigitalInput _indexerNote = new DigitalInput(Constants.kIndexerBeamBreakerId);

	/**
	 * @return Whether there's a note in the indexer according to its beam breaker
	 */
	public static boolean getNoteInIndexer() {
		return !_indexerNote.get();
	}

	/**
	 * @return State of the robot
	 */
	public static RobotStates getRobotState() {
		return robotState;
	}

	/**
	 * Sets the state of the robot to the given state
	 *
	 * @param state - the state to set the robot state to
	 */
	public static void setRobotState(RobotStates state) {
		System.out.println("[Robot State Change] " + robotState.toString() + " -> " + state.toString());
		robotState = state;
		SmartDashboard.putString("Robot State", robotState.toString());
	}

	/**
	 * @return position of the robot
	 */
	public static Pose2d getRobotPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * Set where the code thinks the robot is
	 *
	 * @param pose - the pose to set the robot pose to
	 */
	public static void setRobotPose(Pose2d pose) {
		_robotPosePublisher.set(pose);
		poseEstimator.resetPosition(getGyroYaw(), Swerve.getInstance().getModulePositions(), pose);
	}

	/**
	 * Updates the robot pose according to odometry parameters
	 *
	 * @param modulePositions - The current position of the swerve modules.
	 */
	public static void updateRobotPose(SwerveModulePosition[] modulePositions) {
		poseEstimator.update(getGyroYaw(), modulePositions);

		_robotPosePublisher.set(getRobotPose());
	}

	/**
	 * Updates the robot pose according to the given vision estimation
	 *
	 * @param visionEstimation - the estimation
	 */
	public static void updateRobotPose(VisionEstimation visionEstimation) {
		if (visionEstimation.hasTargets){
			double distanceToTarget = getRobotPose().getTranslation().getDistance(visionEstimation.target.getTranslation());

			poseEstimator.addVisionMeasurement(
				visionEstimation.pose,
				visionEstimation.timestamp,
				new Matrix<>(Nat.N3(), Nat.N1(), new double[] {
					VisionConstants.distanceToFOM(distanceToTarget),
					VisionConstants.distanceToFOM(distanceToTarget),
					VisionConstants.distanceToFOM(distanceToTarget),
				})
			);
		}

		_robotPosePublisher.set(getRobotPose());
	}

	public static void updateRobotPose(VisionEstimation[] visionEstimations) {
		Pose2d averagePose = new Pose2d();
		double averageTimestamp = 0;
		double rotationSum = 0;
		Pose2d averageTarget = new Pose2d();
		int count = 0;

		for (VisionEstimation estimation : visionEstimations) {
			if (estimation.pose == null || !estimation.hasTargets) continue;

			averagePose = new Pose2d(
				averagePose.getX() + estimation.pose.getX(),
				averagePose.getY() + estimation.pose.getY(),
				averagePose.getRotation());
			rotationSum += estimation.pose.getRotation().getDegrees();

			averageTarget = new Pose2d(
				averageTarget.getX() + estimation.target.getX(),
				averageTarget.getY() + estimation.target.getY(),
				new Rotation2d()
			);

			averageTimestamp += estimation.timestamp;
			count++;
		}

		averagePose = new Pose2d(
				averagePose.getX() / count,
				averagePose.getY() / count,
				new Rotation2d(Rotation2d.fromDegrees(rotationSum).getRadians() / count));

		averageTarget = new Pose2d(
			averageTarget.getX() / count,
			averageTarget.getY() / count,
			new Rotation2d());

		if (count == 0) return;

		updateRobotPose(new VisionEstimation(averagePose, averageTimestamp / count, true, averageTarget));
	}

	/**
	 * @return yaw angle of the robot according to gyro
	 */
	public static Rotation2d getGyroYaw() {
		if (!isSimulated())
			return Rotation2d.fromDegrees(SwerveConstants.kInvertGyro ? -navX.getAngle() : navX.getAngle());
		else
			return SwerveConstants.kInvertGyro
					? getRobotPose().getRotation().unaryMinus()
					: getRobotPose().getRotation();
	}

	public static Translation3d getRobotVelocity() {
		return new Translation3d(navX.getVelocityX(), navX.getVelocityY(), navX.getVelocityZ());
	}

	/**
	 * Resets the gyro angle, sets it to the given angle
	 *
	 * @param angle - the angle to set the gyro to
	 */
	public static void resetGyro(Rotation2d angle) {
		if (!isSimulated()) {
			System.out.print("Gyro: " + navX.getAngle() + " -> ");
			navX.reset();
			navX.setAngleAdjustment(angle.getDegrees());
			System.out.println(navX.getAngle());
		} else {
			System.out.print("Gyro: " + getRobotPose().getRotation().getDegrees() + " -> ");
			setRobotPose(new Pose2d(getRobotPose().getTranslation(), angle));
			System.out.println(getRobotPose().getRotation().getDegrees());
		}
	}

	/**
	 * Initialize and create the pose estimator for knowing robot's pose according to vision and
	 * odometry
	 */
	public static void initPoseEstimator() {
		poseEstimator = !isSimulated()
				? new SwerveDrivePoseEstimator(
						SwerveConstants.kSwerveKinematics,
						getGyroYaw(),
						SwerveIO.getInstance().getModulePositions(),
						new Pose2d())
				: new SwerveDrivePoseEstimator(
						SwerveConstants.kSwerveKinematics,
						new Rotation2d(),
						SwerveIO.getInstance().getModulePositions(),
						new Pose2d());
	}

	/**
	 * @return Whether the robot is at simulation mode or deployed on a real robot
	 */
	public static boolean isSimulated() {
		return Robot.isSimulation();
	}

	public static boolean isAutonomous() {
		return isSimulated() ? DriverStationSim.getAutonomous() : DriverStation.isAutonomous();
	}

	public static DriverStation.Alliance getAlliance() {
		return isSimulated()
				? (DriverStationSim.getAllianceStationId().ordinal() > 3
						? DriverStation.Alliance.Blue
						: DriverStation.Alliance.Red)
				: DriverStation.getAlliance().get();
	}

	public static AllianceStationID getAllianceStation() {
		return isSimulated() ? DriverStationSim.getAllianceStationId() : DriverStation.getRawAllianceStation();
	}
}
