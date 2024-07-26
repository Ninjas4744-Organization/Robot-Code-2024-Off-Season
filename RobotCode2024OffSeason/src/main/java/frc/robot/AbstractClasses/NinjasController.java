package frc.robot.AbstractClasses;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.DataClasses.ControllerConstants;

public abstract class NinjasController {
    public enum ControlState {
        PERCENT_OUTPUT,
        MOTION_MAGIC,
        POSITION_PIDF,
        VELOCITY_PIDF
    }

    protected ControlState _controlState;
    protected HashMap<String, GenericEntry> _shuffleboardEnteries;
    protected ControllerConstants _constants;

    public NinjasController(ControllerConstants constants) {
        _constants = constants;
        _controlState = ControlState.PERCENT_OUTPUT;

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
     * @return The current value of the controller according to the control state, could be percent and could be position
     */
    public abstract double get();

    /**
     * Sets the current value of the controller according to the control state, could be percent, could be position and more...
     */
    public abstract void set(double value);

    /**
     * Sets the current control state and sets the value of the controller according to the control state, could be percent, could be position and more...
     */
    public void set(ControlState state, double value){
        setControlState(state);
        set(value);
    }

    /**
     * Stops the controller and all movement
     */
    public abstract void stop();

    /**
     * Sets the control state of the controller
     */
    public void setControlState(ControlState state) {
        _controlState = state;
    }

    /**
     * @return the position encoder value no matter what the control state is
     */
    public abstract double getEncoder();

    /**
     * Sets the position in the encoder
     */
    public abstract void setEncoder(double position);

    /**
     * Resets the encoder, sets it to the home position
     */
    public void resetEncoder() {
        setEncoder(_constants.encoderHomePosition);
    }

    /**
     * @return the PID setpoint, only works if the control state has something to do with PID
     */
    public abstract double getSetpoint();

    /**
     * Updates the shuffleboard values
     */
    protected abstract void updateShuffleboard();
}
