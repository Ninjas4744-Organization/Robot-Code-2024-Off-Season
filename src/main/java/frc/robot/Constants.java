package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.DataClasses.MainControllerConstants;
import frc.robot.DataClasses.PIDFConstants;
import frc.robot.DataClasses.SimulatedControllerConstants;
import frc.robot.DataClasses.SwerveModuleConstants;
import java.io.IOException;
import java.util.HashMap;

public final class Constants {
	public static final int kDriverJoystickPort = 0;
	public static final int kOperatorJoystickPort = 1;
	public static final int kNoteDetectorID = 0;

	public static class ShooterConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 22;
			kControllerConstants.main.inverted = false;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "Shooter";
			kControllerConstants.PIDFConstants = new PIDFConstants(0.018, 0, 0, 60, 120);
			//			kControllerConstants.positionGoalTolerance = 5;
			kControllerConstants.encoderConversionFactor = 7.2;
			kControllerConstants.encoderHomePosition = 0;
			kControllerConstants.gearRatio = 27;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}
	}

	public static class  ElevatorConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 24;
			kControllerConstants.main.inverted = true;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "Elevator";
			kControllerConstants.PIDFConstants = new PIDFConstants(5, 0, 0, 8, 8);
			kControllerConstants.positionGoalTolerance = 0.01;
			kControllerConstants.encoderConversionFactor = 0.0098174;
			kControllerConstants.encoderHomePosition = 0;
			kControllerConstants.gearRatio = 1;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public static final int kLimitSwitchID = 7;

		public class States {
			public static final double kAmp = 0.4;
			public static final double kTrap = 0.5;
			public static final double kClose = 0;
		}
	}

	public static class ClimberConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 24;
			kControllerConstants.main.inverted = true;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "Climber";
			kControllerConstants.PIDFConstants = new PIDFConstants(5, 0, 0, 8, 8);
			kControllerConstants.positionGoalTolerance = 0.01;
			kControllerConstants.encoderConversionFactor = 0.0098174;
			kControllerConstants.encoderHomePosition = 0;
			kControllerConstants.gearRatio = 1;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public static final int kLimitSwitchID = 7;

		public class States {
			public static final double kUp = 0.4;
			public static final double kClose = 0;
		}
	}

	public static class RotationConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 24;
			kControllerConstants.main.inverted = true;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "Rotation";
			kControllerConstants.PIDFConstants = new PIDFConstants(5, 0, 0, 8, 8);
			kControllerConstants.positionGoalTolerance = 0.01;
			kControllerConstants.encoderConversionFactor = 0.0098174;
			kControllerConstants.encoderHomePosition = 0;
			kControllerConstants.gearRatio = 1;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public static final int kLimitSwitchID = 7;

		public class States {
			public static final double kAmp = 0.4;
			public static final double kTrap = 0;
			public static final double kClose = 0;
		}
	}

	public static class RollersConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 24;
			kControllerConstants.main.inverted = true;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "Rollers";
			kControllerConstants.PIDFConstants = new PIDFConstants(5, 0, 0, 8, 8);
			kControllerConstants.positionGoalTolerance = 0.01;
			kControllerConstants.encoderConversionFactor = 0.0098174;
			kControllerConstants.encoderHomePosition = 0;
			kControllerConstants.gearRatio = 1;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public static final int kLimitSwitchID = 7;

		public class States {
			public static final double kIntake = -1;
			public static final double kOutake = 1;
		}
	}
	public static class RollersIntakeConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 24;
			kControllerConstants.main.inverted = true;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "RollersIntake";
			kControllerConstants.PIDFConstants = new PIDFConstants(5, 0, 0, 8, 8);
			kControllerConstants.positionGoalTolerance = 0.01;
			kControllerConstants.encoderConversionFactor = 0.0098174;
			kControllerConstants.encoderHomePosition = 0;
			kControllerConstants.gearRatio = 1;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public static final int kLimitSwitchID = 7;

		public class States {
			public static final double kIntake = -1;
			public static final double kOutake = 1;
		}
	}
	public static class RollersOuttakeConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 24;
			kControllerConstants.main.inverted = true;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "RollersOuttake";
			kControllerConstants.PIDFConstants = new PIDFConstants(5, 0, 0, 8, 8);
			kControllerConstants.positionGoalTolerance = 0.01;
			kControllerConstants.encoderConversionFactor = 0.0098174;
			kControllerConstants.encoderHomePosition = 0;
			kControllerConstants.gearRatio = 1;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public static final int kLimitSwitchID = 7;

		public class States {
			public static final double kIntake = -1;
			public static final double kOutake = 1;
		}
	}

	public static final class SwerveConstants {
		public static final double kSpeedFactor = 0.5;
		public static final double kRotationSpeedFactor = 0.25;
		public static final double kJoystickDeadband = 0.3;

		public static final boolean kInvertGyro = true; // Always ensure Gyro is CCW+ CW-

		/** Whether to drive without velocity PID control */
		public static final boolean kOpenLoop = true;
		/** Whether to drive relative to the field or the robot */
		public static final boolean kFieldRelative = true;

		/** Drivetrain Constants */
		public static final double kTrackWidth = Units.inchesToMeters(30);

		public static final double kWheelBase = Units.inchesToMeters(30);
		public static final double kWheelDiameter = Units.inchesToMeters(4);
		public static final double kWheelCircumference = kWheelDiameter * Math.PI;

		public static final double kOpenLoopRamp = 0.25;
		public static final double kClosedLoopRamp = 0.0;

		public static final double kDriveGearRatio = (6.12 / 1.0); // 5.14:1
		public static final double kAngleGearRatio = (12.8 / 1.0); // 12.8:1

		public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

		/** Swerve compensation */
		public static final double kVoltageComp = 12.0;

		/** Swerve Current limiting */
		public static final int kAngleContinuousCurrentLimit = 20;

		public static final int kDriveContinuousCurrentLimit = 35;

		/* Modules angle PID values */
		public static final double kAngleP = 0.01;
		public static final double kAngleI = 0.0;
		public static final double kAngleD = 0.005;
		public static final double kAngleFF = 0.0;

		/** Modules drive PID values */
		public static final double kDriveP = 0.0;

		public static final double kDriveI = 0.0;
		public static final double kDriveD = 0.0;
		public static final double kDriveFF = 0.0;

		/** Modules drive characterization values */
		public static final double kDriveS = 0.667;

		public static final double kDriveV = 2.44;
		public static final double kDriveA = 0.27;

		/** Drive Motor Conversion Factors */
		public static final double driveConversionPositionFactor = ((kWheelDiameter * Math.PI) / kDriveGearRatio);

		/** Drive Motor Conversion Factors */
		public static final double driveConversionVelocityFactor =
				((kWheelDiameter * Math.PI) / kDriveGearRatio) / 60.0;

		public static final double angleConversionFactor = 360.0 / 12.8;

		/** Swerve Profiling Values */
		public static final double maxSpeed = 6; // meters per second

		public static final double maxAngularVelocity = 12;

		/** Neutral Modes */
		public static final com.revrobotics.CANSparkBase.IdleMode angleNeutralMode =
				com.revrobotics.CANSparkBase.IdleMode.kBrake;

		public static final com.revrobotics.CANSparkBase.IdleMode driveNeutralMode =
				com.revrobotics.CANSparkBase.IdleMode.kBrake;

		/** Motor Inverts */
		public static final boolean driveInvert = true;

		public static final boolean angleInvert = false;

		/** Angle Encoder Invert */
		public static final boolean canCoderInvert = false;

		/* Swerve angle PID values */
		public static final double kSwerveAngleP = 0.0035;
		public static final double kSwerveAngleI = 0;
		public static final double kSwerveAngleD = 0;

		/* Swerve drive assist PID values */
		public static final TrapezoidProfile.Constraints kDriveAssistProfileConstraints =
				new TrapezoidProfile.Constraints(maxSpeed / 3, maxSpeed);
		public static final double kDriveAssistP = 0.1525 * 1.5;
		public static final double kDriveAssistI = 0;
		public static final double kDriveAssistD = 0;

		/* Swerve axis lock PID values */
		public static final double kSwerveAxisLockP = 0.2;
		public static final double kSwerveAxisLockI = 0.0;
		public static final double kSwerveAxisLockD = 0.0;

		/**
		 * Swerve drive assist threshold, if the drive assist angle difference from driver angle is
		 * bigger than this value, the drive assist will be ignored. degrees
		 */
		public static final double kDriveAssistThreshold = 45;

		/** Module Specific Constants */
		/** Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final int driveMotorID = 10;
			public static final int angleMotorID = 11;
			public static final int canCoderID = 30;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
			public static final boolean isDriverEncoderInverted = false;
			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/** Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 12;
			public static final int angleMotorID = 13;
			public static final int canCoderID = 31;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
			public static final boolean isDriverEncoderInverted = false;
			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/** Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 14;
			public static final int angleMotorID = 15;
			public static final int canCoderID = 32;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
			public static final boolean isDriverEncoderInverted = true;

			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/** Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 16;
			public static final int angleMotorID = 17;
			public static final int canCoderID = 33;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
			public static final boolean isDriverEncoderInverted = false;

			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		public class Simulation {
			public static final double kSimToRealSpeedConversion = 0.02; // meters per 0.02s -> meters per 1s
			public static final double kAcceleration = 10;
		}
	}

	public static final class AutoConstants {
		public static final double maxVelocity = 1;
		public static final double maxAcceleration = 1;

		public static final double kPXController = 1;
		public static final double kPYController = 1;
		public static final double kPThetaController = 1;

		public static final double kMaxSpeedMetersPerSecond = 3.6;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
		public static final PathConstraints constraints = new PathConstraints(
				kMaxSpeedMetersPerSecond,
				kMaxAccelerationMetersPerSecondSquared,
				kMaxAngularSpeedRadiansPerSecond,
				kMaxAngularSpeedRadiansPerSecondSquared);
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

		public static final HolonomicPathFollowerConfig pathFollowerConfig =
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
						// your Constants class
						new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
						new PIDConstants(Constants.AutoConstants.kPYController, 0.0, 0.0), // Rotation PID constants
						Constants.AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						SwerveConstants
								.kTrackWidth, // Drive base radius in meters. Distance from robot center to furthest
						// module.
						new ReplanningConfig() // Default path replanning config. See the API for the options
						// here
						);
	}

	public static class VisionConstants {
		public static final double kMaxAmbiguity = 0.7;

		public static HashMap<String, Transform3d> getCamerasPoses() {
			HashMap<String, Transform3d> cameras = new HashMap<>();

			double deg2rad = 0.0174533;
			cameras.put("Front", new Transform3d(-0.35, 0, 0.2775, new Rotation3d(0, 0, 0)));
			cameras.put("BackLeft", new Transform3d(-0.325, 0.175, 0.2075, new Rotation3d(0, 0, 120 * deg2rad)));
			cameras.put("BackRight", new Transform3d(-0.325, -0.175, 0.1875, new Rotation3d(0, 0, -120 * deg2rad)));

			return cameras;
		}

		public static final double kFieldLength = Units.feetToMeters(54.0);
		public static AprilTagFieldLayout kBlueFieldLayout;
		public static AprilTagFieldLayout kRedFieldLayout;

		static {
			try {
				kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

				kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

				kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
				kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
			} catch (IOException e) {
				throw new RuntimeException(e);
			}
		}

		public static AprilTagFieldLayout getFieldLayout() {
			if (Robot.isReal()) {
				if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) return kBlueFieldLayout;
				else return kRedFieldLayout;
			} else return kBlueFieldLayout;
		}

		public class Simulation {
			public static final int kResolutionWidth = 1280;
			public static final int kResolutionHeight = 720;
			public static final double kFOV = 70;
			public static final double kAverageError = 0.3;
			public static final double kErrorStdDev = 0.5;
			public static final int kFPS = 15;
			public static final int kAverageLatency = 35;
			public static final int kLatencyStdDev = 5;
		}

		public static Pose2d getAmpPose() {
			if (Robot.isSimulation() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
				return getFieldLayout().getTagPose(6).get().toPose2d();
			else return getFieldLayout().getTagPose(5).get().toPose2d();
			//			return new Pose2d();
		}

		public static Pose2d getSourcePose() {
			if (Robot.isSimulation() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
				return getFieldLayout().getTagPose(2).get().toPose2d();
			else return getFieldLayout().getTagPose(9).get().toPose2d();
			//			return new Pose2d();
		}

		public static Pose2d getSpeakerPose() {
			if (Robot.isSimulation() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
				return getFieldLayout().getTagPose(7).get().toPose2d();
			else return getFieldLayout().getTagPose(4).get().toPose2d();
			//			return new Pose2d();
		}

		public static Pose2d getTagPose(int id) {
			return getFieldLayout().getTagPose(id).get().toPose2d();
		}

		/**
		 * Get the april tag that the translation between it and the robot is closest to the given direction,
		 * for example if robot is moving to amp, the amp tag will be returned
		 * @param dir The direction to find the closest camera to, field relative
		 * @return The apriltag that is closest by direction
		 */
		public static AprilTag getTagByDirection(Translation2d dir) {
			Rotation2d dirAngle = dir.getAngle();
			AprilTag closestTag = null;
			Rotation2d closestAngleDiff = Rotation2d.fromDegrees(Double.MAX_VALUE);

			for (AprilTag tag : getFieldLayout().getTags()) {
				Rotation2d robotToTagAngle = tag.pose
						.toPose2d()
						.getTranslation()
						.minus(RobotState.getRobotPose().getTranslation())
						.getAngle();

				if (dirAngle.minus(robotToTagAngle).getDegrees() < closestAngleDiff.getDegrees()) {
					closestAngleDiff = dirAngle.minus(robotToTagAngle);
					closestTag = tag;
				}
			}

			return closestTag;
		}

		/**
		 * Get offset april tag pose according to its looking direction
		 * @param offset how much to offset the pose of the tag to its looking direction
		 * @return the pose of the offset tag
		 */
		public static Pose2d getOffsetTagPose(Pose2d tagPose, double offset) {
			Translation2d offsetTranslation =
					new Translation2d(offset, tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(90)));
			return tagPose.transformBy(new Transform2d(offsetTranslation, new Rotation2d()));
		}
	}

	public static class NoteDetectionConstants {
		public static final double limelightMountAngleX = 18.22;
		public static final double limelightMountAngleY = 0;
		public static final double limelightHeight = 0.395;
		public static final double noteHeight = 0.0254;
	}
}
