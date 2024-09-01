package frc.robot.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotState;

public class DriveAssist {
    private PIDController _xPID;
    private PIDController _yPID;
    private TrapezoidProfile _xProfile;
    private TrapezoidProfile _yProfile;
    private Timer _profileTimer;
    
    private boolean isCurrentlyDriveAssisting = false;
    
    private Pose2d _startPose;
    private ChassisSpeeds _startSpeeds;
    
    private PIDController _anglePID;

    public DriveAssist(PIDController anglePID){
        _xProfile = new TrapezoidProfile(Constants.SwerveConstants.kDriveAssistProfileConstraints);
        _yProfile = new TrapezoidProfile(Constants.SwerveConstants.kDriveAssistProfileConstraints);
        _profileTimer = new Timer();

        _xPID = new PIDController(
            Constants.SwerveConstants.kDriveAssistP,
            Constants.SwerveConstants.kDriveAssistI,
            Constants.SwerveConstants.kDriveAssistD);

        _yPID = new PIDController(
            Constants.SwerveConstants.kDriveAssistP,
            Constants.SwerveConstants.kDriveAssistI,
            Constants.SwerveConstants.kDriveAssistD);

        _anglePID = anglePID;
    }

    public DriveAssist(){
        this(new PIDController(0.001, 0, 0));
    }

    /**
     * Calculates the drive assist
     *
     * @param movingDirection - the direction the robot is moving according to the driver input, field
     *     relative
     * @param rotation - the rotation movement of the robot according to the driver input, field
     *     relative
     * @param targetPose - the pose of the closest target(tags and notes), field relative
     * @param isForTags - if the drive assist is for tags and not notes
     * @return Calculated drive assist as chassis speeds(if the rotation is 0, use the rotation from
     *     the drive input), field relative
     */
    public ChassisSpeeds driveAssist(Translation2d movingDirection, double rotation, Pose2d targetPose, boolean isForTags) {
        if (movingDirection.getX() == 0 && movingDirection.getY() == 0) return new ChassisSpeeds(0, 0, rotation);

        Translation2d toTargetDirection =
            targetPose.getTranslation().minus(RobotState.getRobotPose().getTranslation());

        Rotation2d movingAngle = movingDirection.getAngle();
        Rotation2d toTargetAngle = toTargetDirection.getAngle();
        Rotation2d angleDiff = toTargetAngle.minus(movingAngle);

        if (Math.abs(angleDiff.getDegrees()) < Constants.SwerveConstants.kDriveAssistThreshold){
            if(!isCurrentlyDriveAssisting)
                startingDriveAssist();
            isCurrentlyDriveAssisting = true;

            return calculateDriveAssist(toTargetAngle, targetPose, isForTags);
        }
        else
            isCurrentlyDriveAssisting = false;

        return new ChassisSpeeds(movingDirection.getX() * Constants.SwerveConstants.maxSpeed * Constants.SwerveConstants.kSpeedFactor, movingDirection.getY() * Constants.SwerveConstants.maxSpeed * Constants.SwerveConstants.kSpeedFactor, rotation);
    }

    private ChassisSpeeds calculateDriveAssist(Rotation2d toTargetAngle, Pose2d targetPose, boolean isForTags){
        double anglePIDMeasurement = (RobotState.isSimulated() ? -1 : 1) * RobotState.getRobotPose().getRotation().getDegrees();
        double anglePIDSetpoint = (RobotState.isSimulated() ? -1 : 1) * (isForTags
            ? targetPose
            .getRotation()
            .minus(Rotation2d.fromDegrees(180))
            .getDegrees()
            : toTargetAngle.getDegrees());

        return new ChassisSpeeds(
            _xProfile.calculate(_profileTimer.get(), new TrapezoidProfile.State(_startPose.getX(), _startSpeeds.vxMetersPerSecond), new TrapezoidProfile.State(targetPose.getX(), 0)).velocity,
            _yProfile.calculate(_profileTimer.get(), new TrapezoidProfile.State(_startPose.getY(), _startSpeeds.vyMetersPerSecond), new TrapezoidProfile.State(targetPose.getY(), 0)).velocity,
//              _xPID.calculate(RobotState.getRobotPose().getX(), targetPose.getX()) * Constants.SwerveConstants.maxSpeed,
//              _yPID.calculate(RobotState.getRobotPose().getY(), targetPose.getY()) * Constants.SwerveConstants.maxSpeed,

            _anglePID.calculate(anglePIDMeasurement, anglePIDSetpoint)
                * Constants.SwerveConstants.maxAngularVelocity);
    }
    
    private void startingDriveAssist(){
        _startPose = RobotState.getRobotPose();
        _startSpeeds = SwerveIO.getInstance().getChassisSpeeds();
        _profileTimer.restart();
    }

    /**
     * Call me when turning off drive assist.
     * You need to call me when turning off drive assist for some logic going on in here. DON'T ASK!😁
     */
    public void turnOffDriveAssist(){
        isCurrentlyDriveAssisting = false;
    }
}
