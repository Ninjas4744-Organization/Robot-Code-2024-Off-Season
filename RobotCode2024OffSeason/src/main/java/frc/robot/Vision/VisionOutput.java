package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionOutput {
    public Pose2d robotPose;
    public double timestamp;

    public int maxAmbiguityTagId;
    public int farthestTagId;
    public int closestTagId;

    public double maxAmbiguity;
    public double farthestTagDist;
    public double closestTagDist;

    public boolean hasTargets = false;
}