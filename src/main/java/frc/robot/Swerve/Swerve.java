package frc.robot.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.Vision.Vision;
import java.util.Arrays;
import java.util.List;

public class Swerve extends SubsystemBase {
    private static Swerve _instance = null;

    public static Swerve getInstance() {
        if (_instance == null) _instance = new Swerve();
        return _instance;
    }

    private SwerveModule[] _modules;
    private SwerveDriveOdometry _odometry;
    private PIDController _anglePID;
    private PIDController _driveAssistXPID;
    private PIDController _driveAssistYPID;
    private boolean isDriveAssist = false;
    private boolean isAnglePID = false;
    private boolean isBayblade = false;

    /** Creates a new Swerve. */
    private Swerve() {
        _modules =
                new SwerveModule[] {
                    new SwerveModule(0, SwerveConstants.Mod0.constants),
                    new SwerveModule(1, SwerveConstants.Mod1.constants),
                    new SwerveModule(2, SwerveConstants.Mod2.constants),
                    new SwerveModule(3, SwerveConstants.Mod3.constants)
                };

        _odometry =
                new SwerveDriveOdometry(
                        SwerveConstants.kSwerveKinematics, RobotState.getGyroYaw(), getModulePositions());
        resetOdometry(new Pose2d());

        _driveAssistXPID =
                new PIDController(
                        SwerveConstants.kDriveAssistP,
                        SwerveConstants.kDriveAssistI,
                        SwerveConstants.kDriveAssistD);
        _driveAssistYPID =
                new PIDController(
                        SwerveConstants.kDriveAssistP,
                        SwerveConstants.kDriveAssistI,
                        SwerveConstants.kDriveAssistD);
        _anglePID =
                new PIDController(
                        SwerveConstants.kSwerveAngleP,
                        SwerveConstants.kSwerveAngleI,
                        SwerveConstants.kSwerveAngleD);
        _anglePID.enableContinuousInput(
                Rotation2d.fromDegrees(-180).getDegrees(), Rotation2d.fromDegrees(180).getDegrees());
    }

    /**
      * Drives the robot
      *
      * @param translation - speed percentage to move in x and y
      * @param rotation - speed percentage to rotate the robot, positive is counter clockwise
      * @param fieldRelative - whether or not the robot movement is relative to the field or the robot
      *     itself
      * @param isOpenLoop - whether or not to not use velocity PID for the modules
      */
    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        ChassisSpeeds drive =
                new ChassisSpeeds(
                        translation.getX() * SwerveConstants.kSpeedFactor * SwerveConstants.maxSpeed,
                        translation.getY() * SwerveConstants.kSpeedFactor * SwerveConstants.maxSpeed,
                        rotation * SwerveConstants.kRotationSpeedFactor * SwerveConstants.maxAngularVelocity);

        if (isAnglePID)
            drive.omegaRadiansPerSecond =
                    _anglePID.calculate(RobotState.getGyroYaw().getDegrees())
                            * SwerveConstants.maxAngularVelocity;

        if (isDriveAssist && Vision.getInstance().hasTargets("Front")) {
            Pose2d targetPose = Vision.getInstance().getClosestTag("Front").pose.toPose2d();
            drive = calculateDriveAssist(translation, drive.omegaRadiansPerSecond, targetPose, true);
        }

        if (isBayblade) drive.omegaRadiansPerSecond = SwerveConstants.maxAngularVelocity;

        SwerveModuleState[] swerveModuleStates =
                SwerveConstants.kSwerveKinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(drive, RobotState.getGyroYaw())
                                : drive);
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    /**
      * Drives the robot
      *
      * @param chassisSpeeds - robot relative chassis speeds to drive according to
      */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates =
                SwerveConstants.kSwerveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, false);
    }

    /**
      * Sets the modules to the given states
      *
      * @param desiredStates - the wanted state for each module
      */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed);

        for (SwerveModule mod : _modules)
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
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

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
      * Resets the swerve odometry to the given pose so it thinks the robot is at that pose
      *
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

    /** Sets the swerve modules rotation so it makes an X shape and makes the swerve immovable */
    public void setWheelsToX() {
        setModuleStates(
                new SwerveModuleState[] {
                    // front left
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
                    // front right
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
                    // back left
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
                    // back right
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
                },
                false);
    }

    /**
      * @return drive encoder value of each module, angle of each module
      */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];

        for (SwerveModule mod : _modules) positions[mod.moduleNumber] = mod.getPosition();

        return positions;
    }

    /**
      * Resets the swerve modules to their absolute encoders to fix problems with a module no knowing
      * where it is
      */
    public void resetModulesToAbsolute() {
        System.out.println("---------------Reseting modules to absolute---------------");

        for (SwerveModule mod : _modules) mod.resetToAbsolute();

        System.out.println("---------------Reseting modules to absolute---------------");
    }

    /**
      * Auto drives the robot to a given pose with an offset from the target
      *
      * @param targetPose - the pose and rotation to drive to
      * @param offset - the distance from the target to stop at
      * @return following path command
      */
    public Command goTo(Pose2d targetPose, double offset) {
        Pose2d currentPose = RobotState.getRobotPose();

        Translation2d offsetTranslation =
                new Translation2d(
                        offset * targetPose.getRotation().getCos(), offset * targetPose.getRotation().getSin());

        targetPose =
                new Pose2d(
                        targetPose.getX() + offsetTranslation.getX(),
                        targetPose.getY() + offsetTranslation.getY(),
                        targetPose.getRotation());

        SmartDashboard.putNumber("Current X", currentPose.getX());
        SmartDashboard.putNumber("Current Y", currentPose.getY());
        SmartDashboard.putNumber("Current 0", currentPose.getRotation().getDegrees());

        SmartDashboard.putNumber("Auto Target X", targetPose.getX());
        SmartDashboard.putNumber("Auto Target Y", targetPose.getY());
        SmartDashboard.putNumber("Auto Target 0", targetPose.getRotation().getDegrees());

        // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(currentPose, targetPose);
        List<Translation2d> bezierPoints =
                Arrays.asList(
                        currentPose.getTranslation(),
                        currentPose.getTranslation(),
                        targetPose.getTranslation(),
                        targetPose.getTranslation());

        PathPlannerPath path =
                new PathPlannerPath(
                        bezierPoints, AutoConstants.constraints, new GoalEndState(0, targetPose.getRotation()));

        return AutoBuilder.followPath(path);
    }

    /**
      * Makes the swerve use PID to look at the given angle
      *
      * @param angle - the angle to look at
      * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round
      *     the given angle to the nearest 0, 45, 90, 135... it rounds the angle only if the rounded
      *     angle is close enough to the given angle so for example if the given angle is 28 and the
      *     rounded angle is 45 it won't round. if you write 1 as the roundToAngle there will be no
      *     rounding, DON'T USE 0 (division by zero error)
      */
    public void lookAt(double angle, double roundToAngle) {
        double roundedAngle = Math.round(angle / roundToAngle) * roundToAngle;
        angle = Math.abs(roundedAngle - angle) <= roundToAngle / 3 ? roundedAngle : angle;

        _anglePID.setSetpoint(angle);
        isAnglePID = true;
    }

    /**
      * Makes the swerve use PID to look according to the given direction
      *
      * @param direction - the direction vector to look
      * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round
      *     the given angle (calculated from direction) to the nearest 0, 45, 90, 135... it rounds the
      *     angle only if the rounded angle is close enough to the given angle so for example if the
      *     given angle is 28 and the rounded angle is 45 it won't round. if you write 1 as the
      *     roundToAngle there will be no rounding, DON'T USE 0 (division by zero error)
      */
    public void lookAt(Translation2d direction, double roundToAngle) {
        if (!(direction.getX() == 0 && direction.getY() == 0))
            lookAt(direction.getAngle().getDegrees(), roundToAngle);
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
      *
      * @param movingDirection - the direction the robot is moving according to the driver input, field
      *     relative
      * @param rotation - the rotation of the robot according to the driver input, field relative
      * @param targetPose - the pose of the closest target(tags and notes), field relative
      * @param isForTags - if the drive assist is for tags and not notes
      * @return Calculated drive assist as chassis speeds(if the rotation is 0, use the rotation from
      *     the drive input), field relative
      */
    private ChassisSpeeds calculateDriveAssist(
            Translation2d movingDirection, double rotation, Pose2d targetPose, boolean isForTags) {
        if (movingDirection.getX() == 0 && movingDirection.getY() == 0)
            return new ChassisSpeeds(0, 0, rotation);

        Translation2d toTargetDirection =
                targetPose.getTranslation().minus(RobotState.getRobotPose().getTranslation());

        Rotation2d movingAngle = movingDirection.getAngle();
        Rotation2d toTargetAngle = toTargetDirection.getAngle();
        Rotation2d angleDiff = toTargetAngle.minus(movingAngle);

        if (Math.abs(angleDiff.getDegrees()) < SwerveConstants.kDriveAssistThreshold) {
            double anglePIDMeasurement = RobotState.getRobotPose().getRotation().getDegrees();
            double anglePIDSetpoint =
                    isForTags
                            ? targetPose.getRotation().minus(Rotation2d.fromDegrees(180)).getDegrees()
                            : toTargetAngle.getDegrees();

            ChassisSpeeds driveAssist =
                    new ChassisSpeeds(
                            _driveAssistXPID.calculate(RobotState.getRobotPose().getX(), targetPose.getX())
                                    * SwerveConstants.maxSpeed,
                            _driveAssistYPID.calculate(RobotState.getRobotPose().getY(), targetPose.getY())
                                    * SwerveConstants.maxSpeed,
                            _anglePID.calculate(anglePIDMeasurement, anglePIDSetpoint)
                                    * SwerveConstants.maxAngularVelocity);

            SmartDashboard.putNumber(
                    "Drive Assist X Error", targetPose.getX() - RobotState.getRobotPose().getX());
            SmartDashboard.putNumber(
                    "Drive Assist Y Error", targetPose.getY() - RobotState.getRobotPose().getY());
            return driveAssist;
        }

        return new ChassisSpeeds(movingDirection.getX(), movingDirection.getY(), rotation);
    }

    /**
      * Sets whether or not the drive assist is on
      *
      * @param isDriveAssist
      */
    public void setIsDriveAssist(boolean isDriveAssist) {
        System.out.println("Drive Assist Mode: " + this.isDriveAssist + " -> " + isDriveAssist);
        this.isDriveAssist = isDriveAssist;
    }

    /**
      * Sets whether or not the bayblade mode is on, if it is on the swerve will rotate full speed non
      * stop
      *
      * @param isBayblade - true if bayblade mode should be on
      */
    public void setBaybladeMode(boolean isBayblade) {
        System.out.println("Bayblade Mode: " + this.isBayblade + " -> " + isBayblade);
        this.isBayblade = isBayblade;
    }

    /** Logs info about the modules and swerve */
    public void log() {
        // TODO: make this work
    }

    @Override
    public void periodic() {
        _odometry.update(RobotState.getGyroYaw(), getModulePositions());
        RobotState.updateRobotPose(getModulePositions());
    }
}
