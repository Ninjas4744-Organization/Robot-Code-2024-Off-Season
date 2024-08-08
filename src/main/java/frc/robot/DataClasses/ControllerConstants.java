package frc.robot.DataClasses;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class ControllerConstants {
    /** The id of the controller */
    public int id;
    /** Current limit */
    public double currentLimit;
    /** The name of the subsystem which uses this controller */
    public String subsystemName;
    /** Whether to invert the output of the controller */
    public boolean invertOutput;
    
    /** PID constants */
    public PIDConstants pidConstants;
    /** The constraints for feedforwarding */
    public TrapezoidProfile.Constraints constraints;
    
    /** The home position of the system where the limit switch is and is usually 0. when the limit switch is hit the encode will reset to this */
    public double encoderHomePosition;
    /** The encoder value gets multiplied by this number, choose a number that will result the encoder to be in meters */
    public double encoderConversionFactor;
    /** The down soft limit, makes the system unable to move under it */
    public double minSoftLimit;
    /** The up soft limit, makes the system unable to move above it */
    public double maxSoftLimit;

    public int shuffleboardEnteriesSize;
}
