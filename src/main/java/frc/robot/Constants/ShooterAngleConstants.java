package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.NinjasLib.DataClasses.MainControllerConstants;
import frc.robot.NinjasLib.DataClasses.PIDFConstants;
import frc.robot.NinjasLib.DataClasses.SimulatedControllerConstants;
import frc.robot.RobotState;

public class ShooterAngleConstants {
    public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
    public static final SimulatedControllerConstants kSimulatedControllerConstants = new SimulatedControllerConstants();

    static {
        kControllerConstants.main.id = 32;
        kControllerConstants.currentLimit = 50;
        kControllerConstants.subsystemName = "ShooterAngle";

        kControllerConstants.PIDFConstants = new PIDFConstants();
        kControllerConstants.PIDFConstants.kP = 0.15;

        kControllerConstants.positionGoalTolerance = 0.5;
        kControllerConstants.dynamicProfiling = true;

        kControllerConstants.encoderConversionFactor = 1.0 / 300.0 * 360.0;
        kControllerConstants.encoderHomePosition = 31;
        kControllerConstants.isMaxSoftLimit = true;
        kControllerConstants.maxSoftLimit = 72;

        kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
        kSimulatedControllerConstants.motorTorque = 1;
    }

    public static final int kLimitSwitchId = 2;

    public static final Translation3d kSpeakerOffset = new Translation3d(0, 0, 0.74);
    public static final Translation3d kShooterPose = new Translation3d(0, 0, 0.12);
    public static final InterpolatingDoubleTreeMap kAngleMap = new InterpolatingDoubleTreeMap();

    static {
        kAngleMap.put(2.5, 40.3);
        kAngleMap.put(1.6, 51.8);
        kAngleMap.put(2.3, 41.8);
        kAngleMap.put(3.0, 36.2);
        kAngleMap.put(3.6, 29.4);
        kAngleMap.put(4.0, 28.7);
        kAngleMap.put(4.5, 25.5);
    }

    public static Pose3d getSpeakerHolePose() {
        return new Pose3d(
            Constants.VisionConstants.getTagPose(Constants.VisionConstants.getSpeakerTag().ID)
                .getX()
                + kSpeakerOffset.getX() * (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
            Constants.VisionConstants.getTagPose(Constants.VisionConstants.getSpeakerTag().ID)
                .getY()
                + kSpeakerOffset.getY() * (RobotState.getInstance().getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
            Constants.VisionConstants.getTagPose(Constants.VisionConstants.getSpeakerTag().ID)
                .getZ()
                + kSpeakerOffset.getZ(),
            new Rotation3d());
    }

    public static Rotation2d calculateLaunchAngle(Pose3d target) {
        double dist = RobotState.getInstance().getRobotPose()
            .getTranslation()
            .plus(ShooterAngleConstants.kShooterPose.toTranslation2d())
            .getDistance(target.toPose2d().getTranslation());

        double angle = kAngleMap.get(dist);

        double angleClamped = Math.min(Math.max(angle, 37.7), 80);
        return Rotation2d.fromDegrees(angleClamped);
    }

    public enum States {
        Up(56),
        Amp(64),
        Close(31),
        Delivery(45);

        private final int value;

        States(int value) {
            this.value = value;
        }

        public int get(){
            return value;
        }
    }
}