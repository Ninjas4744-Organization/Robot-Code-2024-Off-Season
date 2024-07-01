package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PointWithTime;
import frc.robot.subsystems.VisionIO.VisionIOOutput;

public class Vision extends SubsystemBase {
    private VisionIO[] cameras;
    private Pose2d _robotPose;
    private double _timestamp;
    private boolean _hasTargets;

    private VisionIOOutput[] _outputs;

    /**
     * @param cameras Array of cameras being used
     */
    public Vision(VisionIO[] cameras) {
        this.cameras = cameras;

        _robotPose = new Pose2d();
        _timestamp = 0;
        _outputs = new VisionIOOutput[cameras.length];
    }

    @Override
    public void periodic() {
        Pose2d robotPose = new Pose2d();
        double timestamp = 0;

        int camerasWithTargetsCount = 0;
        for (int i = 0; i < cameras.length; i++) {
            VisionIOOutput output = cameras[i].Update();
            _outputs[i] = output;

            if (output.hasTargets){
                Pose2d currentPose = output.robotPose;
                
                robotPose = new Pose2d(
                    robotPose.getX() + currentPose.getX(),
                    robotPose.getY() + currentPose.getY(),
                    robotPose.getRotation().plus(currentPose.getRotation())
                );

                timestamp += output.timestamp;

                camerasWithTargetsCount++;
            }
        }
        
        _hasTargets = camerasWithTargetsCount > 0;
        if(_hasTargets){
            _robotPose = new Pose2d(
                robotPose.getX() / camerasWithTargetsCount,
                robotPose.getY() / camerasWithTargetsCount,
                robotPose.getRotation().div(camerasWithTargetsCount)
            );

            _timestamp = timestamp / camerasWithTargetsCount;
        }
    }

    public Pose2d getRobotPose() {
        return _robotPose;
    }

    /**
     * Returns the robot pose and the time when this pose was detected
     */
    public PointWithTime estimationSupplier(){
        return new PointWithTime(_robotPose, _timestamp);
    }

    public boolean hasTargets(){
        return _hasTargets;
    }

    public double getClosestTagDistance(int camera) {
        return _outputs[camera].closestTagDist;
    }

    public double getClosestTagId(int camera) {
        return _outputs[camera].closestTagId;
    }

    public double getFarthestTagDistance(int camera) {
        return _outputs[camera].farthestTagDist;
    }

    public double getFarthestTagId(int camera) {
        return _outputs[camera].farthestTagId;
    }

    public double getMaxAmbiguity(int camera) {
        return _outputs[camera].maxAmbiguity;
    }

    public double getMaxAmbiguityTagId(int camera) {
        return _outputs[camera].maxAmbiguityTagId;
    }
}
