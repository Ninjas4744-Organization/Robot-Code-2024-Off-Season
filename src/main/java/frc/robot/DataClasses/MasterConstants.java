package frc.robot.DataClasses;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class MasterConstants {
    /** The id of the controller */
    public ControllerConstants master;
    public ControllerConstants[] slaves;
    /** Current limit */
    public double currentLimit = 60;
    /** The name of the subsystem which uses this controller */
    public String subsystemName = "";
    /** Whether to invert the output of the controller */
    
    /** PID constants */
    public double Kp = 0;
    public double Ki = 0;
    public double KIzone = 0;
    public double Kd = 0;
    public double Kf = 0;
    public int kPositionDeadband = 0; // Ticks

    /** System properties */
    public double kCruiseVelocity = 0; // Units/s
    public double kAcceleration = 0; // Units/s^2
    
    /** The home position of the system where the limit switch is and is usually 0. when the limit switch is hit the encode will reset to this */
    public double encoderHomePosition = 0;

    /** The encoder value gets multiplied by this number, choose a number that will result the encoder to be in meters */
    public double encoderConversionFactor = 1;

    /**wether or not to apply minimum soft limit */
    public boolean isMinSoftLimit = false;

    /** The down soft limit, makes the system unable to move under it */
    public double minSoftLimit = 0;

     /**wether or not to apply maximum soft limit */
    public boolean isMaxSoftLimit = false;

    /** The up soft limit, makes the system unable to move above it */
    public double maxSoftLimit = 0;

    public int shuffleboardEnteriesSize = 3;
    public double kGearRatio = 1;
}


