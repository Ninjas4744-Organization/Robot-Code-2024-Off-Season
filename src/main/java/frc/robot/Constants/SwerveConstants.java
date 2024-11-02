package frc.robot.Constants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.NinjasLib.DataClasses.SwerveModuleConstants;

public final class SwerveConstants {
    public static final double kSpeedFactor = 1;
    public static final double kRotationSpeedFactor = 1;
    public static final double kJoystickDeadband = 0.1;

    public static final Rotation2d kShootingAngleError = Rotation2d.fromDegrees(-7);

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
    public static final double maxSpeed = 5; // meters per second
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
        public static final double kP = 0.05;
        public static final double kI = 0;
        public static final double kIZone = 0;
        public static final double kD = 0;

        public static final double kPTheta = 0.07;
        public static final double kITheta = 0.15;
        public static final double kIZoneTheta = 4;
        public static final double kDTheta = 0.003;

        public static final double kMaxSpeed = 2;
        public static final double kAcceleration = 4;
        public static final double kMaxAngularSpeed = 8;
        public static final double kAngularAcceleration = 16;

        public static final TrapezoidProfile.Constraints kAngleConstraints = new TrapezoidProfile.Constraints(
            Units.radiansToDegrees(kMaxAngularSpeed), Units.radiansToDegrees(kAngularAcceleration));

        public static final PathConstraints kConstraints =
            new PathConstraints(kMaxSpeed, kAcceleration, kMaxAngularSpeed, kAngularAcceleration);

        public static final HolonomicPathFollowerConfig kAutonomyConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(kP, kI, kD),
            new PIDConstants(kPTheta, kITheta, kDTheta),
            SwerveConstants.maxModuleSpeed,
            SwerveConstants.kTrackWidth, // Distance from robot center to the furthest module.
            new ReplanningConfig() // Default path replanning config.
        );
    }
}