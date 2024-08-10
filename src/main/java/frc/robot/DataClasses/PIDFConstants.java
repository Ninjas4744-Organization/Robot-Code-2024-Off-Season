package frc.robot.DataClasses;

public class PIDFConstants {
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kIZone = Double.POSITIVE_INFINITY;
    public double kF = 0;
    public int kPositionDeadband = 0;
    public double kCruiseVelocity = 0;
    public double kAcceleration = 0;

    public PIDFConstants() {}

    public PIDFConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
    public PIDFConstants(double kP, double kI, double kD, double kIZone) {
        this(kP, kI, kD);
        this.kIZone = kIZone;
    }
    
    public PIDFConstants(double kP, double kI, double kD, double kIZone, double kCruiseVelocity, double kAcceleration) {
        this(kP, kI, kD, kIZone);
        this.kCruiseVelocity = kCruiseVelocity;
        this.kAcceleration = kAcceleration;
    }
    
    public PIDFConstants(double kP, double kI, double kD, double kIZone, double kF, int kPositionDeadband, double kCruiseVelocity, double kAcceleration) {
        this(kP, kI, kD, kIZone, kCruiseVelocity, kAcceleration);
        this.kF = kF;
        this.kPositionDeadband = kPositionDeadband;
    }
}
