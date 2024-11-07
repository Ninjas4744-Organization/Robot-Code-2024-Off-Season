package frc.robot.Constants;

import com.ninjas4744.NinjasLib.DataClasses.VisionConstants.NoteDetectionConstants;
import com.ninjas4744.NinjasLib.DataClasses.VisionConstants.SimulationConstants;
import com.ninjas4744.NinjasLib.DataClasses.VisionOutput;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotState;

import java.util.Map;

public class VisionConstants {
    public static final com.ninjas4744.NinjasLib.DataClasses.VisionConstants kVisionConstants = new com.ninjas4744.NinjasLib.DataClasses.VisionConstants();
    static{
        kVisionConstants.cameras = Map.of(
            //"Front", new Transform3d(0, 0, 0, new Rotation3d(0, 30, 0)),
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

        kVisionConstants.maxAmbiguity = 0.2;
        kVisionConstants.maxDistance = 4;
        kVisionConstants.fieldLayoutGetter = FieldConstants::getFieldLayout;

        kVisionConstants.simulationConstants = new SimulationConstants();
        kVisionConstants.simulationConstants.resolutionWidth = 1280;
        kVisionConstants.simulationConstants.robotPoseSupplier = () -> RobotState.getInstance().getRobotPose();
        kVisionConstants.simulationConstants.resolutionHeight = 720;
        kVisionConstants.simulationConstants.FOV = 74;
        kVisionConstants.simulationConstants.averageError = 0.3;
        kVisionConstants.simulationConstants.errorStdDev = 0.5;
        kVisionConstants.simulationConstants.FPS = 30;
        kVisionConstants.simulationConstants.averageLatency = 35;
        kVisionConstants.simulationConstants.latencyStdDev = 5;

        kVisionConstants.noteDetectionConstants = new NoteDetectionConstants();
        kVisionConstants.noteDetectionConstants.limelightMountAngleX = 18.22;
        kVisionConstants.noteDetectionConstants.limelightName = "";
        kVisionConstants.noteDetectionConstants.limelightMountAngleY = 0;
        kVisionConstants.noteDetectionConstants.limelightHeight = 0.395;
        kVisionConstants.noteDetectionConstants.noteHeight = 0.0254;
    }

    public static double calculateFOM(VisionOutput estimation) {
        double distFOM = 0.34 * estimation.closestTagDist - 0.124;
        double speedFOM = 0/*0.2 * RobotState.getInstance().getRobotVelocity().getNorm()*/;

        return distFOM + speedFOM;
    };
}
