package frc.robot.DataClasses;

public class MainControllerConstants {
    /** Controller constants for the main controller in the subsystem */
    public ControllerConstants main;

    /** Controller constants for the controllers that follow the main controller in the subsystem */
    public ControllerConstants[] followers;

    /** Current limit */
    public double currentLimit = 60;

    /** The name of the subsystem which uses this controller */
    public String subsystemName = "";
    
    /** PIDF constants */
    public PIDFConstants PIDFConstants = new PIDFConstants();

    /* The error which is considered atGoal(). if the PIDF error is smaller than this value it will be considered atGoal() */
    /** The position error which is considered atGoal() */
    public double positionGoalTolerance = 0.05;
    /** The velocity error which is considered atGoal() */
    public double velocityGoalTolerance = 0.05;
    
    /** The home position of the system where the limit switch is and is usually 0. when the limit switch is hit the encode will reset to this */
    public double encoderHomePosition = 0;

    /** The encoder value gets multiplied by this number, choose a number that will result the encoder to be in meters / degrees */
    public double encoderConversionFactor = 1;

    /** Wether or not to apply minimum soft limit */
    public boolean isMinSoftLimit = false;

    /** The down soft limit, makes the system unable to move under it */
    public double minSoftLimit = 0;

     /** Wether or not to apply maximum soft limit */
    public boolean isMaxSoftLimit = false;

    /** The up soft limit, makes the system unable to move above it */
    public double maxSoftLimit = 0;

    /** Size of shuffleboard entries for this controller's tab in the dashboard */
    public int shuffleboardEnteriesSize = 3;
}