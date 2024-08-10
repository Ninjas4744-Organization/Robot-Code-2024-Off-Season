package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Swerve.Swerve;
import frc.robot.Vision.VisionEstimation;

public class RobotState {
  public enum State {}

  private static State state;
  private static Pose2d robotPose = new Pose2d();
  private static AHRS navX = new AHRS();
  private static SwerveDrivePoseEstimator poseEstimator;
  private static StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  /**
   * @return state of the robot
   */
  public static State getState() {
    return state;
  }

  /**
   * Sets the state of the robot to the given state
   *
   * @param state - the state to set the robot state to
   * @return nothing
   */
  public static void setState(State state) {
    RobotState.state = state;
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
   * @return nothing
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

    robotPose = poseEstimator.getEstimatedPosition();
    publisher.set(poseEstimator.getEstimatedPosition());
  }

  /**
   * @return yaw angle of the robot according to gyro
   */
  public static Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(
        Constants.Swerve.kInvertGyro ? -navX.getAngle() : navX.getAngle());
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

  public static void initPoseEstimator() {
    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.Swerve.kSwerveKinematics,
            getGyroYaw(),
            Swerve.getInstance().getModulePositions(),
            getRobotPose());
  }
}
