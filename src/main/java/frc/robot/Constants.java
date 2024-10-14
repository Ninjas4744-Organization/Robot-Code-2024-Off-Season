package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.NinjasLib.DataClasses.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public final class Constants {
	public static final int kDriverJoystickPort = 0;
	public static final int kOperatorJoystickPort = 1;
	public static final int kIndexerBeamBreakerId = 0;

	public static class ShooterAngleConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 32;
			kControllerConstants.currentLimit = 50;
			kControllerConstants.subsystemName = "ShooterAngle";
			kControllerConstants.PIDFConstants = new PIDFConstants();
			kControllerConstants.PIDFConstants.kCruiseVelocity = 20;
			kControllerConstants.PIDFConstants.kAcceleration = 50;
			kControllerConstants.PIDFConstants.kP = 0.3;
			kControllerConstants.positionGoalTolerance = 1;
			kControllerConstants.encoderConversionFactor = 1.0 / 300.0 * 360.0;
			kControllerConstants.isMaxSoftLimit = true;
			kControllerConstants.maxSoftLimit = 78;
			kControllerConstants.isMinSoftLimit = false;
			kControllerConstants.minSoftLimit = 42;
			kControllerConstants.encoderHomePosition = 40;
			kControllerConstants.dynamicProfiling = false;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public static final int kLimitSwitchId = 2;

		public static final Translation3d kAmpOffset = new Translation3d(0, -0.1, -0.35);
		public static final Translation3d kSpeakerOffset = new Translation3d(0, 0, 0.74);
		public static final Translation3d kShooterPose = new Translation3d(0, 0, 0.135);

		public static Pose3d getAmpHolePose() {
			return new Pose3d(
				VisionConstants.getAmpTag().pose.getX() + kAmpOffset.getX() * (RobotState.getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
				VisionConstants.getAmpTag().pose.getY() + kAmpOffset.getY() * (RobotState.getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
				VisionConstants.getAmpTag().pose.getZ() + kAmpOffset.getZ(),
				new Rotation3d()
			);
		}

		public static Pose3d getSpeakerHolePose() {
			return new Pose3d(
				VisionConstants.getSpeakerTag().pose.getX() + kSpeakerOffset.getX() * (RobotState.getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
				VisionConstants.getSpeakerTag().pose.getY() + kSpeakerOffset.getY() * (RobotState.getAlliance() == DriverStation.Alliance.Red ? -1 : 1),
				VisionConstants.getSpeakerTag().pose.getZ() + kSpeakerOffset.getZ(),
				new Rotation3d()
			);
		}

		public static double getEnterAngle(Translation3d translation) {
			return Units.radiansToDegrees(Math.atan(translation.getZ() / translation.toTranslation2d().getNorm()));
		}

		public static Rotation2d calculateLaunchAngle(Pose3d target) {
			double dist = RobotState.getRobotPose()
				.getTranslation()
				.plus(ShooterAngleConstants.kShooterPose.toTranslation2d())
				.getDistance(target.toPose2d().getTranslation());

			SmartDashboard.putNumber("Dist", dist);
			SmartDashboard.putNumber("Trigo Angle", ShooterAngleConstants.getEnterAngle(new Translation3d(
				target.getX() - (RobotState.getRobotPose().getX() + ShooterAngleConstants.kShooterPose.getX()),
				target.getY() - (RobotState.getRobotPose().getY() + ShooterAngleConstants.kShooterPose.getY()),
				target.getZ() - ShooterAngleConstants.kShooterPose.getZ()
			)));
			NetworkTableInstance.getDefault().getTable("Speaker Hole").getEntry("Speaker Hole").setDoubleArray(new double[] { target.getX(), target.getY(), target.getZ() });

			double angle = Ballistics.findLaunchAngle(
				target.getZ(),
				ShooterAngleConstants.kShooterPose.getZ(),
				dist,
				ShooterAngleConstants.getEnterAngle(new Translation3d(
					target.getX() - (RobotState.getRobotPose().getX() + ShooterAngleConstants.kShooterPose.getX()),
					target.getY() - (RobotState.getRobotPose().getY() + ShooterAngleConstants.kShooterPose.getY()),
					target.getZ() - ShooterAngleConstants.kShooterPose.getZ()
				)));

			double angleClamped = Math.min(Math.max(angle, 40), 80);
			return Rotation2d.fromDegrees(angleClamped);
		}
	}

	public static class ShooterConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 30;
			kControllerConstants.main.inverted = true;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "Shooter";
			kControllerConstants.PIDFConstants = new PIDFConstants();
			kControllerConstants.PIDFConstants.kS = 0.185;
			kControllerConstants.PIDFConstants.kV = 0.117;
			kControllerConstants.PIDFConstants.kCruiseVelocity = 50;
			kControllerConstants.PIDFConstants.kAcceleration = 100;
			kControllerConstants.velocityGoalTolerance = 2;

			kControllerConstants.followers = new ControllerConstants[]{new ControllerConstants()};
			kControllerConstants.followers[0].id = 31;
			kControllerConstants.followers[0].inverted = true;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public static double shooterSpeedToNoteSpeed(double rps) {
			// Constants
			double g = 9.81; // Acceleration due to gravity (m/s^2)
			double wheelRadius = Units.inchesToMeters(2);
			double noteMass = 0.233;
			double frictionCoefficient = 1.5;
			double interactionTime = 1 / 15.0;

			// Calculate angular velocity for both wheels (in radians per second)
			double angularVelocity = 2 * Math.PI * rps;

			// Calculate maximum linear speeds at the edge of the wheels
			double maxSpeed = angularVelocity * wheelRadius;

			// Calculate the frictional force (assuming normal force = weight of the game piece)
			double normalForce = noteMass * g;
			double frictionForce = frictionCoefficient * normalForce;

			// Calculate the effective speed considering the friction over the interaction time
			double distanceTraveled = (maxSpeed * interactionTime) - (0.5 * frictionForce * interactionTime);

			// Calculate the final speed of the game piece

			return distanceTraveled / interactionTime;
		}

		public static double noteSpeedToShooterSpeed(double noteSpeed) {
			// Constants
			double g = 9.81; // Acceleration due to gravity (m/s^2)
			double wheelRadius = Units.inchesToMeters(2);
			double noteMass = 0.233;
			double frictionCoefficient = 1.5;
			double frictionForce = frictionCoefficient * noteMass * g;

			// Solve for rps based on the final speed
			return (noteSpeed + 0.5 * frictionForce) / (2 * Math.PI * wheelRadius);
		}

		public static double calculateLaunchSpeed(double shootAngle, Pose3d target) {
			double dist = RobotState.getRobotPose()
				.getTranslation()
				.plus(ShooterAngleConstants.kShooterPose.toTranslation2d())
				.getDistance(target.toPose2d().getTranslation());

			SmartDashboard.putNumber("Note Shoot Speed", Ballistics.findLaunchSpeed(
				ShooterAngleConstants.kShooterPose.getZ(),
				target.getZ(),
				dist,
				shootAngle
			));

			return noteSpeedToShooterSpeed(Ballistics.findLaunchSpeed(
				ShooterAngleConstants.kShooterPose.getZ(),
				target.getZ(),
				dist,
				shootAngle
			));
		}

		public class States {
			public static final double kSpeaker = 87;
			public static final double kAmp = 30;
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
			kControllerConstants.PIDFConstants = new PIDFConstants();
			kControllerConstants.positionGoalTolerance = 0.01;
			kControllerConstants.encoderConversionFactor = 0.0098174;
			kControllerConstants.encoderHomePosition = 0;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.gearRatio = 1;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public class States {
			public static final double kUp = 0.4;
			public static final double kClose = 0;
		}
	}

	public static class IndexerConstants {
		public static final MainControllerConstants kControllerConstants = new MainControllerConstants();
		public static final SimulatedControllerConstants kSimulatedControllerConstants =
				new SimulatedControllerConstants();

		static {
			kControllerConstants.main.id = 20;
			kControllerConstants.currentLimit = 40;
			kControllerConstants.subsystemName = "Indexer";
			kControllerConstants.positionGoalTolerance = 0.01;
			kControllerConstants.encoderConversionFactor = 0.0098174;
			kControllerConstants.encoderHomePosition = 0;

			kSimulatedControllerConstants.mainControllerConstants = kControllerConstants;
			kSimulatedControllerConstants.motorTorque = 1;
		}

		public class States {
			public static final double kIntake = -1;
			public static final double kIndex = -1;
			public static final double kIndexBack = 1;
			public static final double kShoot = -1;
			public static final double kRoll = 1;
		}
	}

	public static final class SwerveConstants {
		public static final double kSpeedFactor = 0.5;
		public static final double kRotationSpeedFactor = 0.25;
		public static final double kJoystickDeadband = 0.3;

		public static final boolean kInvertGyro = false; // Always ensure Gyro is CCW+ CW-

		/** Whether to drive without velocity PID control */
		public static final boolean kOpenLoop = true;
		/** Whether to drive relative to the field or the robot */
		public static final boolean kFieldRelative = true;

		/** Drivetrain Constants */
		public static final double kTrackWidth = 0.62;

		public static final double kWheelBase = 0.62;
		public static final double kWheelDiameter = Units.inchesToMeters(4);
		public static final double kWheelCircumference = kWheelDiameter * Math.PI;

		public static final double kOpenLoopRamp = 0.25;
		public static final double kClosedLoopRamp = 0.0;

		public static final double kDriveGearRatio = (6.12); // 5.14:1
		public static final double kAngleGearRatio = (12.8); // 12.8:1

		public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
				new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

		/** Swerve compensation */
		public static final double kVoltageComp = 12.0;

		/** Swerve Current limiting */
		public static final int kAngleContinuousCurrentLimit = 50;

		public static final int kDriveContinuousCurrentLimit = 50;

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

		/**
		 * Max speed the swerve could possibly drive
		 */
		public static final double maxSpeed = 4; // meters per second
		/** Max speed the swerve could possibly rotate */
		public static final double maxAngularVelocity = 10.7;
		/**
		 * Max speed a swerve module could possibly drive on ground
		 */
		public static final double maxModuleSpeed = 4.7;

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

		/**
		 * Swerve drive assist distance threshold, if the robot distance from target is
		 * bigger than this value, the drive assist will be ignored. meters
		 */
		public static final double kPathFollowerDistThreshold = 2;

		/* Module Specific Constants */
		/** Front Left Module - Module 0 */
		public static final class Mod0 {
			public static final int driveMotorID = 10;
			public static final int angleMotorID = 11;
			public static final int canCoderID = 40;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
			public static final boolean isDriverEncoderInverted = false;
			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/** Front Right Module - Module 1 */
		public static final class Mod1 {
			public static final int driveMotorID = 12;
			public static final int angleMotorID = 13;
			public static final int canCoderID = 41;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
			public static final boolean isDriverEncoderInverted = false;
			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/** Back Left Module - Module 2 */
		public static final class Mod2 {
			public static final int driveMotorID = 14;
			public static final int angleMotorID = 15;
			public static final int canCoderID = 42;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
			public static final boolean isDriverEncoderInverted = true;

			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		/** Back Right Module - Module 3 */
		public static final class Mod3 {
			public static final int driveMotorID = 16;
			public static final int angleMotorID = 17;
			public static final int canCoderID = 43;
			public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
			public static final boolean isDriverEncoderInverted = false;

			public static final SwerveModuleConstants constants =
					new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
		}

		public class Simulation {
			public static final double kSimToRealSpeedConversion = 0.02; // meters per 0.02s -> meters per 1s
			public static final double kAcceleration = 10;
		}

		public static final class AutoConstants {
			public static final double kP = 2.3;
			public static final double kI = 0;
			public static final double kD = 1;
			public static final double kPTheta = 0.1;
			//		public static final double kPTheta = 0.025;
			public static final double kITheta = 0;
			public static final double kDTheta = 0;

			public static final double kMaxSpeed = 2;
			public static final double kAcceleration = 1;
			public static final double kMaxAngularSpeed = 8;
			public static final double kAngularAcceleration = 16;

			public static final PathConstraints kConstraints =
					new PathConstraints(kMaxSpeed, kAcceleration, kMaxAngularSpeed, kAngularAcceleration);

			public static final HolonomicPathFollowerConfig kAutonomyConfig = new HolonomicPathFollowerConfig(
					new PIDConstants(kP, 0, 0),
					new PIDConstants(kPTheta, 0, 0),
					SwerveConstants.maxModuleSpeed,
					SwerveConstants.kTrackWidth, // Distance from robot center to the furthest module.
					new ReplanningConfig() // Default path replanning config.
					);
		}
	}

	public static class VisionConstants {
		public static final Map<String, Transform3d> kCameras = Map.of(
				"Front", new Transform3d(-0.35, 0, 0.2775, new Rotation3d(0, 0, 0)),
				"BackLeft", new Transform3d(-0.325, 0.175, 0.2075, new Rotation3d(0, 0, Units.degreesToRadians(120))),
				"BackRight",
						new Transform3d(-0.325, -0.175, 0.1875, new Rotation3d(0, 0, Units.degreesToRadians(-120))));

		public static final double kMaxAmbiguity = 0.2;

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
				System.out.println("Id " + tag.ID + ": x" + Math.round(tag.pose.getX() * 100) / 100 + ", y"
						+ Math.round(tag.pose.getY() * 100) / 100 + ", z" + Math.round(tag.pose.getZ() * 100) / 100
						+ ", theta"
						+ Math.round(tag.pose.getRotation().toRotation2d().getDegrees()));
			System.out.println("----------------------------------------------------------------------------");
		}

		public static AprilTagFieldLayout getFieldLayout(List<Integer> ignoredTags) {
			AprilTagFieldLayout layout;

			if (RobotState.isSimulated()) layout = kUseOurField ? kOurFieldLayout : kBlueFieldLayout;
			else {
				if (kUseOurField) layout = kOurFieldLayout;
				else
					layout = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
							? kBlueFieldLayout
							: kRedFieldLayout;
			}

			if (!ignoredTags.isEmpty()) layout.getTags().removeIf(tag -> ignoredTags.contains(tag.ID));

			return layout;
		}

		public static AprilTagFieldLayout getFieldLayout() {
			return getFieldLayout(List.of());
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

		public static AprilTag getAmpTag() {
			if (RobotState.getAlliance() == DriverStation.Alliance.Blue)
				return getFieldLayout().getTags().get(6 - 1);
			else return getFieldLayout().getTags().get(5 - 1);
		}

		public static AprilTag getSourceTag() {
			if (RobotState.getAlliance() == DriverStation.Alliance.Blue)
				return getFieldLayout().getTags().get(2 - 1);
			else return getFieldLayout().getTags().get(9 - 1);
		}

		public static AprilTag getSpeakerTag() {
			if (RobotState.getAlliance() == DriverStation.Alliance.Blue)
				return getFieldLayout().getTags().get(7 - 1);
			else return getFieldLayout().getTags().get(4 - 1);
		}

		public static Pose2d getTagPose(int id) {
			return getFieldLayout().getTagPose(id).get().toPose2d();
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
								.getDistance(RobotState.getRobotPose().getTranslation())
						> distLimit) continue;

				Rotation2d robotToTagAngle = tag.pose
						.toPose2d()
						.getTranslation()
						.minus(RobotState.getRobotPose().getTranslation())
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

	public static class NoteDetectionConstants {
		public static final double limelightMountAngleX = 18.22;
		public static final double limelightMountAngleY = 0;
		public static final double limelightHeight = 0.395;
		public static final double noteHeight = 0.0254;
	}
}
