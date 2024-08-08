package frc.robot.AbstractClasses;

import java.util.HashMap;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.DataClasses.MasterConstants;

public abstract class NinjasController {
    public enum ControlState {
        PERCENT_OUTPUT,
        MOTION_MAGIC,
        PIDF_POSITION,
        PIDF_VELOCITY
    }

    protected ControlState _controlState;
    protected HashMap<String, GenericEntry> _shuffleboardEnteries;
    protected MasterConstants _constants;
    protected double _demand;

    /**
     * Creates a new Ninjas controller
     * @param constants - the constants for the controller
     * @param followersConstants - the constants for the controllers that follow the main controller, optional
     */
    public NinjasController(MasterConstants constants ) {
        _constants = constants;
        _controlState = ControlState.PERCENT_OUTPUT;
        
        _shuffleboardEnteries = new HashMap<>();
        _shuffleboardEnteries.put("position", Shuffleboard.getTab(_constants.subsystemName)
            .add("Position", 0)
            .withWidget("Graph")
            .withSize(_constants.shuffleboardEnteriesSize, constants.shuffleboardEnteriesSize)
            .getEntry());

        _shuffleboardEnteries.put("velocity", Shuffleboard.getTab(_constants.subsystemName)
            .add("Velocity", 0)
            .withWidget("Graph")
            .withSize(_constants.shuffleboardEnteriesSize, constants.shuffleboardEnteriesSize)
            .getEntry());

        _shuffleboardEnteries.put("output", Shuffleboard.getTab(_constants.subsystemName)
            .add("Output", 0)
            .withWidget("Graph")
            .withSize(_constants.shuffleboardEnteriesSize, constants.shuffleboardEnteriesSize)
            .getEntry());
            
        _shuffleboardEnteries.put("setpoint", Shuffleboard.getTab(_constants.subsystemName)
            .add("Setpoint", 0)
            .withWidget("Number Bar")
            .withSize(_constants.shuffleboardEnteriesSize / 2, constants.shuffleboardEnteriesSize)
            .getEntry());
            
        _shuffleboardEnteries.put("controlState", Shuffleboard.getTab(_constants.subsystemName)
            .add("Control State", 0)
            .withWidget("Text View")
            .withSize(_constants.shuffleboardEnteriesSize, _constants.shuffleboardEnteriesSize / 2)
            .getEntry());
    }

    /**
     * Sets percetage output to the controller
     * 
     * @param percent - how much to power the motor between -1 and 1
     * @see #setPosition(double) 
     * @see #setVelocity(double)
     * @see #stop()
     */
    public abstract void setPercent(double percent);

    /**
     * Sets position setpoint to the controller
     * 
     * @param position - the wanted position of the controller according to the encoder
     * @see #setPercent(double) 
     * @see #setVelocity(double)
     * @see #stop()
     */
    public abstract void setPosition(double position);

    /**
     * Sets velocity setpoint output to the controller
     * 
     * @param velocity - the wanted velocity of the controller according to the encoder
     * @see #setPercent(double) 
     * @see #setPosition(double)
     * @see #stop()
     */
    public abstract void setVelocity(double velocity);

    /**
     * Stops the controller of all movement
     * @see #setPercent(double) 
     * @see #setPosition(double)
     * @see #setVelocity(double)
     */
    public void stop(){
        setPercent(0);
    }

    /**
     * @return the position and velocity of the controller
     */
    public abstract State getEncoder();

    /**
     * @return the percent output of the controller
     */
    public abstract double getOutput();

    /**
     * Sets the position in the encoder so it thinks it is at that position
     * 
     * @param position - the position to set the encoder to
     */
    public abstract void setEncoder(double position);

    /**
     * Resets the encoder, sets it to the home position
     */
    public void resetEncoder() {
        setEncoder(0);
    }

    /**
     * @return the setpoint/reference of the controller, the target of PIDF / PID / Motion Magic...
     */
    public abstract TrapezoidProfile.State getSetpoint();

    /**
     * @return wheter or not the controller is at the setpoint, the target of PIDF / PID / Motion Magic...
     */
    public abstract boolean atSetpoint();

    /**
     * Sets the error which is considered atSetpoint(). if the PIDF error is smaller than this value it will be considered atSetpoint()
     * @param positionTolerance - Position error which is considered atSetpoint().
     * @param velocityTolerance - Velocity error which is considered atSetpoint().
     * @see #atSetpoint()
     */
    public void setSetpointTolerance(double positionTolerance, double velocityTolerance){
        // _pidfController.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Updates the shuffleboard values
     */
    protected void updateShuffleboard(){
        _shuffleboardEnteries.get("position").setDouble(getEncoder().position);
        _shuffleboardEnteries.get("velocity").setDouble(getEncoder().velocity);
        _shuffleboardEnteries.get("output").setDouble(getOutput());

        if(_controlState == ControlState.PIDF_POSITION)
            _shuffleboardEnteries.get("setpoint").setDouble(getSetpoint().position);
        else if(_controlState == ControlState.PIDF_VELOCITY)
            _shuffleboardEnteries.get("setpoint").setDouble(getSetpoint().velocity);

        _shuffleboardEnteries.get("controlState").setString(_controlState.name());
    }

    /**
     * Runs controller periodic tasks, run it on the subsystem periodic
     */
    public void periodic() {
        updateShuffleboard();
    }
}
