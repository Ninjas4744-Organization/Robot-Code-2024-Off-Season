package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.DataClasses.VisionEstimation;
import frc.robot.Swerve.Swerve;

public class RobotState {
	public enum RobotStates {
		IDLE,
		PREPARE_INTAKE,
		INTAKE_READY,
		INTAKE,
		PREPARE_AMP_OUTAKE,
		AMP_OUTAKE_READY,
		PREPARE_TRAP_OUTAKE,
		TRAP_OUTAKE_READY,
		OUTAKE,
		CLOSE,
		RESET,
		PREPARE_SHOOT,
		SHOOT_READY,
		SHOOT,
		NOTE_SEARCH,
		HOLDING_NOTE,
		PREPARE_CLIMB,
		CLIMB_READY,
		CLIMB,
		CLIMBED
	}

	private static RobotStates robotState = RobotStates.IDLE;
	private static Pose2d robotPose = new Pose2d();
	private static AHRS navX = new AHRS();
	private static SwerveDrivePoseEstimator poseEstimator;
	private static StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
			.getStructTopic("MyPose", Pose2d.struct)
			.publish();
	private static StructPublisher<Pose2d> publisherVisionEstimations = NetworkTableInstance.getDefault()
			.getStructTopic("VisionTargets", Pose2d.struct)
			.publish();
	private static DigitalInput noteDetector = new DigitalInput(Constants.kNoteDetectorID);

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
		robotState = state;
		SmartDashboard.putString("Robot State", robotState.toString());
	}

	/**
	 * @return position of the robot according to odometry and vision
	 */
	public static Pose2d getRobotPose() {
		return robotPose;
	}

	/**
	 * Sets the robotPose variable to the given pose and updates the publisher
	 *
	 * @param pose - the pose to set the robotPose to
	 */
	public static void setRobotPose(Pose2d pose) {
		robotPose = pose;
		publisher.set(pose);
		poseEstimator.resetPosition(getGyroYaw(), Swerve.getInstance().getModulePositions(), pose);
	}

	/**
	 * Updates the robot pose according to odometry parameters
	 *
	 * @param modulePositions - The current position of the swerve modules.
	 */
	public static void updateRobotPose(SwerveModulePosition[] modulePositions) {
		poseEstimator.update(getGyroYaw(), modulePositions);

		robotPose = poseEstimator.getEstimatedPosition();
		publisher.set(poseEstimator.getEstimatedPosition());
	}

	/**
	 * Updates the robot pose according to the given vision estimation
	 *
	 * @param visionEstimation - the estimation
	 */
	public static void updateRobotPose(VisionEstimation visionEstimation) {
		if (visionEstimation.hasTargets)
			poseEstimator.addVisionMeasurement(visionEstimation.pose, visionEstimation.timestamp);
		publisherVisionEstimations.set(visionEstimation.pose);

		robotPose = poseEstimator.getEstimatedPosition();
		publisher.set(poseEstimator.getEstimatedPosition());
	}

	/**
	 * @return yaw angle of the robot according to gyro
	 */
	public static Rotation2d getGyroYaw() {
		if(!isSimulated())
			return Rotation2d.fromDegrees(SwerveConstants.kInvertGyro ? -navX.getAngle() : navX.getAngle());
		else
			return getRobotPose().getRotation();
	}

	/**
	 * Resets the gyro angle, sets it to the given angle
	 *
	 * @param angle - the angle to set the gyro to
	 */
	public static void resetGyro(Rotation2d angle) {
		System.out.print("Gyro: " + navX.getAngle() + " -> ");
		navX.reset();
		navX.setAngleAdjustment(angle.getDegrees());
		System.out.println(navX.getAngle());
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
						Swerve.getInstance().getModulePositions(),
						getRobotPose())
				: new SwerveDrivePoseEstimator(
						SwerveConstants.kSwerveKinematics,
						new Rotation2d(),
						Swerve.getInstance().getModulePositions(),
						robotPose);
	}

	/**
	 * @return Whether there is a note in the robot
	 */
	public static boolean hasNote() {
		return isSimulated() || noteDetector.get();
	}

	public static boolean isSimulated() {
		return Robot.isSimulation();
	}

	public static boolean atAmp() {
		return RobotState.getRobotPose()
						.getTranslation()
						.getDistance(VisionConstants.getAmpPose().getTranslation())
				< 0.5;
	}

	public static boolean atSource() {
		return RobotState.getRobotPose()
						.getTranslation()
						.getDistance(VisionConstants.getSourcePose().getTranslation())
				< 0.5;
	}

	public static boolean atSpeaker() {
		return RobotState.getRobotPose()
						.getTranslation()
						.getDistance(VisionConstants.getSpeakerPose().getTranslation())
				< 2.5;
	}
}
