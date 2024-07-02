package frc.robot.Vision;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionEstimation {
    /** The pose of the robot */
    public Pose2d pose;

    /** The time this pose was detected by */
    public double timestamp;

    /** Whether or not this the camera that returned this estimation has targets */
    public boolean hasTargets;
}
