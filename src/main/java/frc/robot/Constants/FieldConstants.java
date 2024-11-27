package frc.robot.Constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class FieldConstants {
    public static final boolean kUseOurField = false;
    public static final double kFieldLength = Units.feetToMeters(54.0);
    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;
    public static AprilTagFieldLayout kOurFieldLayout;

    static {
        try {
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);

            double ourFieldLength = 7.3;
            double ourFieldWidth = 6.4;
            double ourTagHeight = 1.6;
            List<AprilTag> tags = new ArrayList<AprilTag>();
            tags.add(new AprilTag(
                1, new Pose3d(new Translation3d(0, ourFieldWidth / 2, ourTagHeight), new Rotation3d(0, 0, 0))));
            tags.add(new AprilTag(
                2,
                new Pose3d(
                    new Translation3d(ourFieldLength / 2, ourFieldWidth, ourTagHeight),
                    new Rotation3d(0, 0, -0.5 * Math.PI))));
            tags.add(new AprilTag(
                3,
                new Pose3d(
                    new Translation3d(ourFieldLength, ourFieldWidth / 2, ourTagHeight),
                    new Rotation3d(0, 0, Math.PI))));
            tags.add(new AprilTag(
                4,
                new Pose3d(new Translation3d(ourFieldLength / 2, 0, 2.2), new Rotation3d(0, 0, Math.PI / 2))));

            kOurFieldLayout = new AprilTagFieldLayout(tags, ourFieldLength, ourFieldWidth);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        System.out.println("----------------------------------------------------------------------------");
        for (AprilTag tag : getFieldLayout().getTags())
            System.out.println("Id " + tag.ID + ": x" + Math.round(tag.pose.getX() * 100) / 100.0 + ", y"
                + Math.round(tag.pose.getY() * 100) / 100.0 + ", z" + Math.round(tag.pose.getZ() * 100) / 100.0
                + ", theta"
                + Math.round(tag.pose.getRotation().toRotation2d().getDegrees()));
        System.out.println("----------------------------------------------------------------------------");
    }

    public static AprilTagFieldLayout getFieldLayout(List<Integer> ignoredTags) {
        AprilTagFieldLayout layout;

        if (kUseOurField) layout = kOurFieldLayout;
        else
            layout = RobotState.getInstance().getAlliance() == DriverStation.Alliance.Blue
                ? kBlueFieldLayout
                : kRedFieldLayout;

        if (!ignoredTags.isEmpty()) layout.getTags().removeIf(tag -> ignoredTags.contains(tag.ID));

        return layout;
    }

    public static AprilTagFieldLayout getFieldLayout() {
        return getFieldLayout(List.of());
    }

    public static AprilTag getAmpTag() {
        if (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Blue)
            return getFieldLayout().getTags().get(6 - 1);
        else return getFieldLayout().getTags().get(5 - 1);
    }

    public static AprilTag getSourceTag() {
        if (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Blue)
            return getFieldLayout().getTags().get(2 - 1);
        else return getFieldLayout().getTags().get(9 - 1);
    }

    public static AprilTag getSpeakerTag() {
        if (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Blue)
            return getFieldLayout().getTags().get(7 - 1);
        else return getFieldLayout().getTags().get(4 - 1);
    }

    public static Pose3d getTagPose(int id) {
        return getFieldLayout().getTagPose(id).get();
    }

    /**
     * Get the april tag that the translation between it and the robot is closest to the given direction,
     * for example if robot is moving to amp, the amp tag will be returned
     * @param dir The direction to find the closest camera to, field relative
     * @param distLimit Limit distance so far away tags will be disqualified
     * @return The apriltag that is closest by direction
     */
    public static AprilTag getTagByDirection(Translation2d dir, double distLimit) {
        Rotation2d dirAngle = dir.getAngle();
        AprilTag closestTag = null;
        Rotation2d closestAngleDiff = Rotation2d.fromDegrees(Double.MAX_VALUE);

        for (AprilTag tag : getFieldLayout().getTags()) {
            if (tag.pose
                .toPose2d()
                .getTranslation()
                .getDistance(RobotState.getInstance().getRobotPose().getTranslation())
                > distLimit) continue;

            Rotation2d robotToTagAngle = tag.pose
                .toPose2d()
                .getTranslation()
                .minus(RobotState.getInstance().getRobotPose().getTranslation())
                .getAngle();

            if (Math.abs(dirAngle.minus(robotToTagAngle).getDegrees()) < Math.abs(closestAngleDiff.getDegrees())) {
                closestAngleDiff = dirAngle.minus(robotToTagAngle);
                closestTag = tag;
            }
        }

        return closestTag;
    }

    /**
     * Get the april tag that the translation between it and the robot is closest to the given direction,
     * for example if robot is moving to amp, the amp tag will be returned
     *
     * @param dir The direction to find the closest camera to, field relative
     * @return The apriltag that is closest by direction
     */
    public static AprilTag getTagByDirection(Translation2d dir) {
        return getTagByDirection(dir, Double.MAX_VALUE);
    }

    /**
     * Get offset april tag pose according to its looking direction
     * @param offset how much to offset the pose of the tag to its looking direction
     * @return the pose of the offset tag
     */
    public static Pose2d getOffsetTagPose(Pose2d tagPose, double offset) {
        //			Translation2d offsetTranslation =
        //					new Translation2d(offset, tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(0)));
        return tagPose.transformBy(new Transform2d(offset, 0, new Rotation2d()));
    }
}
