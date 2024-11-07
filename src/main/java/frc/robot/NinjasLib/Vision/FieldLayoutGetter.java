package frc.robot.NinjasLib.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

import java.util.List;

@FunctionalInterface
public interface FieldLayoutGetter {
    AprilTagFieldLayout getFieldLayout(List<Integer> ignoredTags);
}
