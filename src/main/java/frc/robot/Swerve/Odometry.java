package frc.robot.Swerve;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.WheelPositions;

/**
 * Odometry class with IMU-based correction, linearly blending IMU velocity and encoder velocity.
 *
 * @param <T> Wheel positions type.
 */
public class Odometry<T extends WheelPositions<T>> {
  private final Kinematics<?, T> m_kinematics;
  private Pose2d m_poseMeters;

  private Rotation2d m_gyroOffset;
  private Rotation2d m_previousAngle;
  private T m_previousWheelPositions;


  /**
   * Constructs an Odometry object.
   *
   * @param kinematics The kinematics of the drivebase.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param initialPoseMeters The starting position of the robot on the field.
   */
  public Odometry(
      Kinematics<?, T> kinematics,
      Rotation2d gyroAngle,
      T wheelPositions,
      Pose2d initialPoseMeters) {
    m_kinematics = kinematics;
    m_poseMeters = initialPoseMeters;
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousAngle = m_poseMeters.getRotation();
    m_previousWheelPositions = wheelPositions.copy();
  }

  /**
   * Resets the robot's position on the field.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param poseMeters The position on the field that your robot is at.
   */
  public void resetPosition(Rotation2d gyroAngle, T wheelPositions, Pose2d poseMeters) {
    m_poseMeters = poseMeters;
    m_previousAngle = m_poseMeters.getRotation();
    m_gyroOffset = m_poseMeters.getRotation().minus(gyroAngle);
    m_previousWheelPositions = wheelPositions.copy();
  }

  /**
   * Updates the robot's position on the field using a combination of encoder data and IMU data.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelPositions The current encoder readings.
   * @param encoderVelocity The current velocity from the wheel encoders (Translation2d).
   * @param imuVelocity The current velocity from the IMU (Translation2d).
   * @return The new pose of the robot.
   */
  public Pose2d update(Rotation2d gyroAngle, T wheelPositions, Translation2d encoderVelocity, Translation2d imuVelocity) {
    // Calculate the robot's current angle based on the gyro and its offset.
    var angle = gyroAngle.plus(m_gyroOffset);

    // Calculate the delta between encoder velocity and IMU velocity
    double velocityDelta = encoderVelocity.minus(imuVelocity).getNorm();
    // Calculate the IMU displacement using the provided velocity and deltaTime
    Translation2d imuDisplacement = new Translation2d(
        imuVelocity.getX() * 1,
        imuVelocity.getY() * 1
    );

    Twist2d encodersDis = m_kinematics.toTwist2d(m_previousWheelPositions, wheelPositions);

    // Calculate a weight based on velocity delta; linearly increase reliance on IMU as the delta grows
    double weight = Math.min(1.0, velocityDelta);

    // Compute the blended velocity (combining encoder and IMU data)
    double blendedDx = encodersDis.dx * (1.0 - weight) + imuDisplacement.getX() * weight;
    double blendedDy = encodersDis.dy * (1.0 - weight) + imuDisplacement.getY() * weight;

    // Update dtheta using gyro data directly (angular velocity)
    double dtheta = angle.minus(m_previousAngle).getRadians();

    // Use the blended velocities and gyro angle to create a new twist (change in pose)
    var twist = new Twist2d(blendedDx, blendedDy, dtheta);

    // Update the pose using the twist and the previous pose
    var newPose = m_poseMeters.exp(twist);

    // Store previous state for the next update
    m_previousWheelPositions = wheelPositions.copy();
    m_previousAngle = angle;
    m_poseMeters = new Pose2d(newPose.getTranslation(), angle);

    return m_poseMeters;
  }
}
