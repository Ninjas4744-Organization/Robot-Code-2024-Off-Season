package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.lib.util.PointWithTime;
import frc.robot.Constants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Swerve extends SubsystemBase {

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private AHRS gyro;
  private Supplier<PointWithTime> _estimationSupplier;
  private BooleanSupplier _hasVisionMeasurement;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();

  private Field2d m_field_solution = new Field2d();
  public SwerveDrivePoseEstimator _estimator;

  public Swerve(Supplier<PointWithTime> estimationSupplier, BooleanSupplier hasVisionMeasurement) {
    log_modules();
    gyro = new AHRS();
    
    
    _estimationSupplier = estimationSupplier;
    _hasVisionMeasurement = hasVisionMeasurement;

    // Creates all four swerve modules into a swerve drive
    mSwerveMods = new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Mod0.constants),
      new SwerveModule(1, Constants.Swerve.Mod1.constants),
      new SwerveModule(2, Constants.Swerve.Mod2.constants),
      new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    // creates new swerve odometry (odometry is where the robot is on the field)
    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());
    resetOdometry(new Pose2d());
    _estimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), new Pose2d());

    AutoBuilder.configureHolonomic(
        this::getLastCalculatedPosition, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
            new PIDConstants(Constants.AutoConstants.kPYController, 0.0, 0.0), // Rotation PID constants
            Constants.AutoConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
            Constants.Swerve.trackWidth, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          Optional<Alliance> alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)
  // takes the coordinate on field wants to go to, the rotation of it, whether or
  // not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        // fancy way to do an if else statement
        // if field relative == true, use field relative stuff, otherwise use robot
        // centric
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    // sets to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void drive(ChassisSpeeds targetSpeed)
  // takes the coordinate on field wants to go to, the rotation of it, whether or
  // not in field relative mode, and if in open loop control
  {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeed,
        new Translation2d());
    // sets to top speed if above top speed
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    // set states for all 4 modules
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false);// MIGHT NEED TO CHANGE
    }
  }

  public ChassisSpeeds getChassisSpeeds() {

    return Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public void setWheelsToX() {
    setModuleStates(new SwerveModuleState[] {
        // front left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
        // front right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
        // back left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
        // back right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public void zeroModules(){
    System.out.println("---------------Reseting to Absolute---------------");
    for(SwerveModule mod : mSwerveMods)
      mod.resetToAbsolute();
    System.out.println("---------------Reseting to Absolute---------------");
  }

  public Rotation2d getYaw() {
    // fancy if else loop again
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : gyro.getRotation2d();
  }

  public Pose2d getLastCalculatedPosition() {
    return _estimator.getEstimatedPosition();
  }

  // private void updatePV() {
  //   for (Optional<EstimatedRobotPose> estimation : _estimationsSupplier.get()) {

  //     estimation.ifPresent(pose -> {

  //       _estimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);

  //     });
  //   }
  // }

  public void log_modules() {
    // Shuffleboard.getTab("Debug").add("gyro", getYaw().getDegrees());
    // for (SwerveModule mod : mSwerveMods) {
    //   Shuffleboard.getTab("Debug").add(
    //       "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
    //   Shuffleboard.getTab("Debug").add(
    //       "Mod " + mod.moduleNumber + " Integrated",
    //       mod.getState().angle.getDegrees());
    //   Shuffleboard.getTab("Debug").add(
    //       "Mod " + mod.moduleNumber + " Velocity",
    //       mod.getState().speedMetersPerSecond);
    // }
    // SmartDashboard.putNumber("gyro", getYaw().getDegrees());

    // for (SwerveModule mod : mSwerveMods) {

    //   SmartDashboard.putNumber(

    //       "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());

    //   SmartDashboard.putNumber(

    //       "Mod " + mod.moduleNumber + " Integrated",

    //       mod.getState().angle.getDegrees());

    //   SmartDashboard.putNumber(

    //       "Mod " + mod.moduleNumber + " Velocity",

    //       mod.getState().speedMetersPerSecond);

    // }
  }

  @Override
  public void periodic() {
    publisher.set(getLastCalculatedPosition());
    _estimator.update(getYaw(), getPositions());

    if(_hasVisionMeasurement.getAsBoolean() && _estimationSupplier.get() != null)
      _estimator.addVisionMeasurement(_estimationSupplier.get().getPoint(),_estimationSupplier.get().getTime());

    swerveOdometry.update(getYaw(), getPositions());
    m_field_solution.setRobotPose(getLastCalculatedPosition());

    // log_modules();
  }
}