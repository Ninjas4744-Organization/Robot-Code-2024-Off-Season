package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.NoteDetectionConstants;
import frc.robot.RobotState;

public class NoteDetection {
  private static StructPublisher<Pose2d> publisher =
      NetworkTableInstance.getDefault().getStructTopic("NotePose", Pose2d.struct).publish();

  /**
   * @return The pose of the note on the field
   */
  public static Pose2d getNotePose() {
    double tx = LimelightHelpers.getTX("");
    double ty = LimelightHelpers.getTY("");

    Translation2d translation = convertMeasurement(tx, ty);

    SmartDashboard.putNumber("Dist X", translation.getX());
    SmartDashboard.putNumber("Dist Y", translation.getY());

    Transform2d noteTransform =
        new Transform2d(
            new Translation2d(translation.getY(), -translation.getX())
                .rotateBy(RobotState.getGyroYaw()),
            new Rotation2d());

    Pose2d notePose = RobotState.getRobotPose().plus(noteTransform);
    publisher.set(notePose);
    return notePose;
  }

  /** Converts limelight's degrees measurement to meters */
  private static Translation2d convertMeasurement(double tx, double ty) {
    double distY =
        (NoteDetectionConstants.limelightHeight - NoteDetectionConstants.noteHeight)
            / Math.tan(
                Rotation2d.fromDegrees(NoteDetectionConstants.limelightMountAngleX + ty)
                    .getRadians());

    double distX =
        Math.tan(
                Rotation2d.fromDegrees(NoteDetectionConstants.limelightMountAngleY + tx)
                    .getRadians())
            * distY;

    return new Translation2d(distX, distY);
  }

  /**
   * @return Whether the limelight has detected a note
   */
  public static boolean hasTarget() {
    SmartDashboard.putBoolean("Limelight detected", LimelightHelpers.getTA("") > 0.5);
    return LimelightHelpers.getTA("") > 0.5;
  }
}
