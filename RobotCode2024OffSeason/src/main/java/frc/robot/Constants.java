package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
  public static final int kJoystickPort = 0;
  public static final int kJoystick2Port = 1;
  public static final int kCurrentLimit = 40;

  public static final class Climber {
    public static final int kMotor1ID = 21;
    public static final int kMotor2ID = 20;
    public static final int kLimitSwitchID = 2;

    public static final double kMaxClimber = 0.4;
    public static final double kTrapChainHeight = 0.35;

    public static final class ControlConstants {
      public static final double kGearRatio = 27;
      public static final double kConversionPosFactor = 0.0023295454545455;
      public static final double kConversionVelFactor = kConversionPosFactor / 60;

      public static final double kP = 12;
      public static final double kI = 0;
      public static final double kD = 0;

      public static final double kMaxVelocity = 30;
      public static final double kMaxAcceleration = 60;
      public static final TrapezoidProfile.Constraints kConstraints = new Constraints(kMaxVelocity, kMaxAcceleration);
    }
  }

  public static final class Rotation {
    public static final int kMotorID = 22;
    public static final int kLimitSwitchID = 9;

    public final static class States {
      public static final double kUpRotation = 120;
      public static final double kSourceOpenRotation = 70;
      public static final double kAmpOpenRotation = 0;
      public static final double kTrapOpenRotation = 87;
    }

    public static final class ControlConstants {
      public static final double kGearRatio = 27;
      public static final double kConversionPosFactor = /* 360 / kGearRatio */7.2;
      public static final double kConversionVelFactor = kConversionPosFactor / 60;

      public static final double kP = 0.0185;
      public static final double kI = 0.0002;
      public static final double kD = 0;

      public static final double kMaxVelocity = 60;
      public static final double kMaxAcceleration = kMaxVelocity * 2;
      public static final TrapezoidProfile.Constraints kConstraints = new Constraints(kMaxVelocity, kMaxAcceleration);
    }
  }

  public static final class Rollers {
    public static final int kMotorID = 23;
    public static final int kBeamBreakerID = 5;
    public static final double kTimeToOutake = 1;
    // public static final double kTimeToIntake = 0.25;
  }

  public static final class Elevator {
    public static final int kMotorID = 24;
    public static final int kLimitSwitchID = 7;

    public static final class States {
      // public static final double kRealZero = -0.015;
      public static final double kSourceOpenHeight = 0.1;
      public static final double kAmpOpenHeight = 0.4;
      public static final double kTrapOpenHeight = 0.38;
    }

    public static final class ControlConstants {
      public static final double kGearRatio = 16.0;
      public static final double kWinchDiameter = 50 / 10 / 100.0;// milimiters/centimeters/meter
      public static final double kConversionPosFactor = (kWinchDiameter * Math.PI) / kGearRatio;
      public static final double kConversionVelFactor = (kWinchDiameter * Math.PI) / kGearRatio / 60;

      public static final double kP = 4.5;
      public static final double kI = 0.0017;
      public static final double kD = 0;

      public static final double kMaxVelocity = 6;
      public static final double kMaxAcceleration = kMaxVelocity * 2;
      public static final TrapezoidProfile.Constraints kConstraints = new Constraints(kMaxVelocity, kMaxAcceleration);
    }
  }

  public static final class Swerve {
    public static final double kDriveCoefficient = 1;
    public static final double kDriveRotationCoefficient = 0.5;
    public static final double kActionCoefficient = 0.25;
    public static final double stickDeadband = 0.05;

    // public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(30);
    public static final double wheelBase = Units.inchesToMeters(30);
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0); // 5.14:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 35;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.005;
    public static final double angleKFF = 0.0;

    /* Angle PID Values */
    public static final double smartAngleKP = 0.2;
    public static final double smartAngleKI = 0;
    public static final double smartAngleKD = 2;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.0;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = ((wheelDiameter * Math.PI) / driveGearRatio);
    /* Drive Motor Conversion Factors */
    public static final double driveConversionVelocityFactor = ((wheelDiameter * Math.PI) / driveGearRatio) / 60.0;

    public static final double angleConversionFactor = 360.0 / 12.8;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final com.revrobotics.CANSparkBase.IdleMode angleNeutralMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;
    public static final com.revrobotics.CANSparkBase.IdleMode driveNeutralMode = com.revrobotics.CANSparkBase.IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 11;
      public static final int canCoderID = 30;
      public static final Rotation2d angleOffset = 
      Rotation2d.fromDegrees(0);
      public static final boolean isDriverEncoderInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 12;
      public static final int angleMotorID = 13;
      public static final int canCoderID = 31;
      public static final Rotation2d angleOffset = 
      Rotation2d.fromDegrees(0);
      public static final boolean isDriverEncoderInverted = false;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 14;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 32;
      public static final Rotation2d angleOffset = 
      Rotation2d.fromDegrees(0);
      public static final boolean isDriverEncoderInverted = true;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 16;
      public static final int angleMotorID = 17;
      public static final int canCoderID = 33;
      public static final Rotation2d angleOffset = 
      Rotation2d.fromDegrees(0);
      public static final boolean isDriverEncoderInverted = false;

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double maxVelocity = 3.0/3;
    public static final double maxAcceleration = 3.0;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static final double kMaxSpeedMetersPerSecond = 3.6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration,
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Constraint for the motion profilied robot angle controller
  }

  public static final class pathFollowingConstants {
    public static final double maxVelocity = 1;
    public static final double maxAcceleration = 1.25;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI*2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI*2;
    public static final PathConstraints constraints = new PathConstraints(maxVelocity, maxAcceleration,
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class VisionConstants {
    public static final double maxAmbiguity = 0.7;
  }
}