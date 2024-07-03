package frc.robot.DataClasses;

import com.pathplanner.lib.util.PIDConstants;

public class ControllerConstants {
    /** The id of the controller */
    public int Id;
    /** Current limit */
    public double currentLimit;
    /** The name of the subsystem which uses this controller */
    public String subsystemName;
    
    /** PID constants */
    public PIDConstants pidConstants;
    
    /** The maximum velocity to move at when using feedforward */
    public double cruiseVelocity;
    /** The acceleration to move at when using feedforward */
    public double acceleration;
    
    /** The home position of the system, usually where the limit switch is and usually 0 */
    public double encoderHomePosition;
    /** The encoder value gets multiplied by this number, choose a number that will result the encoder to be in meters */
    public double encoderConversionFactor;
    /** The down soft limit, makes the system unable to move under it */
    public double minSoftLimit;
    /** The up soft limit, makes the system unable to move above it */
    public double maxSoftLimit;

    public int shuffleboardEnteriesSize;
}
