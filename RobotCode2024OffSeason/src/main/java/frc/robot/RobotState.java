package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class RobotState {
    public enum State{

    }

    private static State state;

    private static Pose2d robotPose;

    private static AHRS navX = new AHRS();

    private static StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

    /**
     * @return state of the robot
     */
    public static State getState() {
        return state;
    }

    /**
     * Sets the state of the robot to the given state
     * @param state - the state to set the robot state to
     * @return nothing
     */
    public static void setState(State state) {
        RobotState.state = state;
    }

    /**
     * @return position of the robot according to odometry and vision
     */
    public static Pose2d getRobotPose() { 
        return robotPose;
    }

    /**
     * Sets the robotPose variable to the given pose and updates the publisher
     * @param pose - the pose to set the robotPose to
     * @return nothing
     */
    public static void setRobotPose(Pose2d pose) {
        robotPose = pose;
        publisher.set(pose);
    }

    /**
     * @return yaw angle of the robot according to gyro
     */
    public static Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(Constants.Swerve.kInvertGyro ? 360 - navX.getAngle() : navX.getAngle());
    }

    /**
     * Resets the gyro angle, sets it to the given angle
     * @param angle - the angle to set the gyro to
     */
    public static void resetGyro(Rotation2d angle) {
        navX.setAngleAdjustment(-navX.getAngle() + angle.getDegrees());
    }
}
