package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    double limelightMountAngleY = 90 - 71.78290480217515;
    double limelightHeight = 0.395;
    double targetHeight = 0.0254;
    double distY =
        (limelightHeight - targetHeight)
            / Math.tan(Rotation2d.fromDegrees(limelightMountAngleY + ty).getRadians());

    double limelightMountAngleX = 0;
    double distX = Math.tan(Rotation2d.fromDegrees(limelightMountAngleX + tx).getRadians()) * distY;
    SmartDashboard.putNumber("Dist X", distX);
    SmartDashboard.putNumber("Dist Y", distY);

    Transform2d noteTransform =
        new Transform2d(
            new Translation2d(distY, -distX).rotateBy(RobotState.getGyroYaw()), new Rotation2d());

    Pose2d notePose = RobotState.getRobotPose().plus(noteTransform);
    publisher.set(notePose);
    return notePose;
  }

  /**
   * @return Whether the limelight has detected a note
   */
  public static boolean hasTarget() {
    SmartDashboard.putBoolean("Limelight detected", LimelightHelpers.getTA("") > 0.5);
    return LimelightHelpers.getTA("") > 0.5;
  }
}
