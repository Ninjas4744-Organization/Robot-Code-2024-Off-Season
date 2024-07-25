package frc.robot.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Vision.Vision;

import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

public class Swerve extends SubsystemBase {
  private static Swerve _instance = null;

  public static Swerve getInstance() {
    if(_instance == null)
      _instance = new Swerve();
    return _instance;
  }

  private SwerveModule[] _modules;
  private SwerveDriveOdometry _odometry;
  private PIDController _swerveAnglePID;
  private PIDController _driveAssistPID;
  private boolean isDriveAssist = false;
  private boolean isAnglePID = false;
  private boolean isBayblade = false;

  /**
   * Creates a new Swerve.
   */
  private Swerve() {
    _modules = new SwerveModule[] {
      new SwerveModule(0, Constants.Swerve.Mod0.constants),
      new SwerveModule(1, Constants.Swerve.Mod1.constants),
      new SwerveModule(2, Constants.Swerve.Mod2.constants),
      new SwerveModule(3, Constants.Swerve.Mod3.constants)
    };

    _odometry = new SwerveDriveOdometry(Constants.Swerve.kSwerveKinematics, RobotState.getGyroYaw(), getModulePositions());
    resetOdometry(new Pose2d());

    _driveAssistPID = new PIDController(Constants.Swerve.kDriveAssistP, Constants.Swerve.kDriveAssistI, Constants.Swerve.kDriveAssistD);
    _swerveAnglePID = new PIDController(Constants.Swerve.kSwerveAngleP, Constants.Swerve.kSwerveAngleI, Constants.Swerve.kSwerveAngleD);
    _swerveAnglePID.enableContinuousInput(-1.5 * Math.PI, 0.5 * Math.PI);
  }

  /**
   * Drives the robot
   * @param translation - speed in m/s to move in x and y
   * @param rotation - speed in m/s to rotate the robot, positive is counter clockwise
   * @param fieldRelative - whether or not the robot movement is relative to the field or the robot itself
   * @param isOpenLoop - whether or not to use velocity PID for the modules
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    translation = translation.times(Constants.Swerve.kSpeedFactor).times(Constants.Swerve.maxSpeed);
    rotation = rotation * Constants.Swerve.kRotationSpeedFactor * Constants.Swerve.maxAngularVelocity;

    if(isAnglePID)
      rotation = _swerveAnglePID.calculate(RobotState.getGyroYaw().getDegrees());

    if(isBayblade)
      rotation = Constants.Swerve.maxAngularVelocity;

    //TODO: add vision and stuff so                   |----------------closest tag or note pose-------------|
    if(isDriveAssist && Vision.getInstance().hasTargets("Front")){
      Pose2d targetPose = Vision.getInstance().getClosestTag("Front").pose.toPose2d();
      translation = calculateDriveAssist(translation, targetPose);
    }

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.kSwerveKinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, RobotState.getGyroYaw().unaryMinus())
          : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
    );
  
    setModuleStates(swerveModuleStates);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds, new Translation2d());
    setModuleStates(swerveModuleStates);
  }

  /**
   * Sets the modules to the given states
   * @param desiredStates - the wanted state for each module
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : _modules)
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
  }

  /**
   * @return array of module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : _modules) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public ChassisSpeeds getChassisSpeeds(){
    return Constants.Swerve.kSwerveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Resets the swerve odometry to the given pose so it thinks the robot is at that pose
   * @param pose - the pose to reset the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    _odometry.resetPosition(RobotState.getGyroYaw(), getModulePositions(), pose);
  }

  /**
   * @return current pose of the robot according to odometry
   */
  public Pose2d getOdometryPosition() {
    return _odometry.getPoseMeters();
  }

  /**
   * Sets the swerve modules rotation so it makes an X shape and makes the swerve immovable
   */
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

  /**
   * @return drive encoder value of each module, angle of each module
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : _modules)
      positions[mod.moduleNumber] = mod.getPosition();

    return positions;
  }

  /**
   * Resets the swerve modules to their absolute encoders to fix problems with a module no knowing where it is
   */
  public void resetModulesToAbsolute(){
    System.out.println("---------------Reseting modules to absolute---------------");

    for(SwerveModule mod : _modules)
      mod.resetToAbsolute();

    System.out.println("---------------Reseting modules to absolute---------------");
  }

  /**
   * Auto drives the robot to a given pose with an offset from the target
   * @param targetPose - the pose and rotation to drive to
   * @param offset - the distance from the target to stop at
   * @return following path command
   */
  public Command goTo(Pose2d targetPose, double offset) {
    Pose2d currentPos = RobotState.getRobotPose();

    Translation2d offsetTranslation = new Translation2d(
      offset * Math.cos(targetPose.getRotation().getRadians()),
      offset * Math.sin(targetPose.getRotation().getRadians())
    );

    targetPose = new Pose2d(targetPose.getX() + offsetTranslation.getX(), targetPose.getY() + offsetTranslation.getY(), targetPose.getRotation());

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(currentPos, targetPose);

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      Constants.pathFollowingConstants.constraints,
      new GoalEndState(0, targetPose.getRotation())
    );
    
    return AutoBuilder.followPath(path);
  }

  /**
   * Makes the swerve use PID to look at the given angle
   * @param angle - the angle to look at
   * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round the given angle to the nearest 0, 45, 90, 135...
   * it rounds the angle only if the rounded angle is close enough to the given angle so for example if the given angle is 28 and the rounded angle is 45 it won't round.
   * if you write 1 as the roundToAngle there will be no rounding, DON'T USE 0 (Division by zero error)
   */
  public void lookAt(double angle, double roundToAngle) {
    double roundedAngle = Math.round(angle / roundToAngle) * roundToAngle;
    angle = Math.abs(roundedAngle - angle) <= roundToAngle / 3 ? roundToAngle : angle;

    _swerveAnglePID.setSetpoint(angle);
    isAnglePID = true;
  }

  /**
   * Makes the swerve use PID to look according to the given direction
   * @param direction - the direction vector to look
   * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round the given angle (calculated from direction) to the nearest 0, 45, 90, 135...
   * it rounds the angle only if the rounded angle is close enough to the given angle so for example if the given angle is 28 and the rounded angle is 45 it won't round.
   * if you write 1 as the roundToAngle there will be no rounding, DON'T USE 0 (Division by zero error)
   */
  public void lookAt(Translation2d direction, double roundToAngle) {
    double angle = 90 - Math.atan2(direction.getY(), direction.getX());
    lookAt(angle, roundToAngle);
  }

  /**
   * Turns off the angle PID so the swerve rotates according to given speed in drive function.
   * running lookAt will turn on the angle PID again
   */
  public void turnOffAnglePID() {
    isAnglePID = false;
  }

  /**
   * Calculates the drive assist
   * @param movingDirection - the direction the robot is moving from the driver, field relative
   * @param targetPose - the pose of the closest target(tags and notes), field relative
   * @return Calculated drive assist moving direction, field relative
   */
  private Translation2d calculateDriveAssist(Translation2d movingDirection, Pose2d targetPose) {
    if(movingDirection.getX() == 0 && movingDirection.getY() == 0)
      return movingDirection;

    Translation2d toTargetDirection = targetPose.getTranslation().minus(RobotState.getRobotPose().getTranslation());
    // toTargetDirection = new Translation2d(
    //   toTargetDirection.getX() / toTargetDirection.getNorm() * Constants.Swerve.maxSpeed / 4,
    //   toTargetDirection.getY() / toTargetDirection.getNorm() * Constants.Swerve.maxSpeed / 4
    // );

    Rotation2d movingAngle = movingDirection.getAngle();
    Rotation2d toTargetAngle = toTargetDirection.getAngle();
    Rotation2d angleDiff = toTargetAngle.minus(movingAngle);

    if(Math.abs(angleDiff.getDegrees()) < Constants.Swerve.kDriveAssistThreshold){
      Translation2d driveAssist = new Translation2d(
        _driveAssistPID.calculate(RobotState.getRobotPose().getX(), targetPose.getX()),
        _driveAssistPID.calculate(RobotState.getRobotPose().getY(), targetPose.getY())
      );

      SmartDashboard.putNumber("Drive Assist X", driveAssist.getX());
      SmartDashboard.putNumber("Drive Assist Y", driveAssist.getY());
    
      return driveAssist;
    }

    return movingDirection;
  }

  /**
   * Sets whether or not the drive assist is on
   * @param isDriveAssist
   */
  public void setIsDriveAssist(boolean isDriveAssist) {
    System.out.println("Drive Assist: " + this.isDriveAssist + " -> " + isDriveAssist);
    this.isDriveAssist = isDriveAssist;
  }

  /**
   * Sets whether or not the bayblade mode is on, if it is on the swerve will rotate full speed non stop
   * @param isBayblade - true if bayblade mode should be on
   */
  public void setBaybladeMode(boolean isBayblade) {
    this.isBayblade = isBayblade;
  }

  /**
   * Logs info about the modules and swerve
   */
  public void log() {
    //TODO: make this work
  }

  @Override
  public void periodic() {
    _odometry.update(RobotState.getGyroYaw(), getModulePositions());
    RobotState.updateRobotPose(getModulePositions());
  }
}