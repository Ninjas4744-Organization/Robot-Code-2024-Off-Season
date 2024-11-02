package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;

import java.util.Map;

public class VisionConstants {
    public static final Map<String, Transform3d> kCameras = Map.of(
//				"Front", new Transform3d(0, 0, 0, new Rotation3d(0, 30, 0)),
        "Front", new Transform3d(0.28 - 0.11 - 0.2, 0.105, -0.055, new Rotation3d(0, 30, 0)),
        "Left",
        new Transform3d(
            -0.035 + 0.1,
            0.285 - 0.33,
            -0.06,
            new Rotation3d(0, 30, Units.degreesToRadians(90 + 3))),
        "Right",
        new Transform3d(
            -0.03 - 0.1,
            -0.285 + 0.33,
            -0.06,
            new Rotation3d(0, 30, Units.degreesToRadians(-90 + 3))));

    public static final double kMaxAmbiguity = 0.2;

    public static double calculateFOM(double distance) {
//			double distFOM = 0.015 * distance * distance + 0.172 * distance - 0.05;
        double distFOM = 0.314 * distance - 0.236;
        double speedFOM = 0.2 * RobotState.getInstance().getRobotVelocity().getNorm();

        return distFOM + speedFOM;
    }

    public static class Simulation {
        public static final int kResolutionWidth = 1280;
        public static final int kResolutionHeight = 720;
        public static final double kFOV = 70;
        public static final double kAverageError = 0.3;
        public static final double kErrorStdDev = 0.5;
        public static final int kFPS = 15;
        public static final int kAverageLatency = 35;
        public static final int kLatencyStdDev = 5;
    }

    public static class NoteDetectionConstants {
        public static final double limelightMountAngleX = 18.22;
        public static final double limelightMountAngleY = 0;
        public static final double limelightHeight = 0.395;
        public static final double noteHeight = 0.0254;
    }
}
