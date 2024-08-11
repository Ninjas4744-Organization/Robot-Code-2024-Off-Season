package frc.robot.AbstractClasses;

import java.util.HashMap;


import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.DataClasses.MainControllerConstants;
import java.util.HashMap;

public abstract class NinjasController {
    public enum ControlState {
        PERCENT_OUTPUT,
        MOTION_MAGIC_POSITION,
        MOTION_MAGIC_VELOCITY,
        PIDF_POSITION,
        PIDF_VELOCITY
    }
    private final int shuffleboardEnteriesSize = 3;
    protected ControlState _controlState = ControlState.PERCENT_OUTPUT;
    protected HashMap<String, GenericEntry> _shuffleboardEnteries = new HashMap<>();
    protected double _demand;
    

    /**
     * Creates a new Ninjas controller
     * @param constants - the constants for the controller
     */
    public NinjasController(MainControllerConstants constants) {
        
        
        _shuffleboardEnteries.put("position", Shuffleboard.getTab(constants.subsystemName)
            .add("Position", 0)
            .withWidget("Graph")
            .withSize(shuffleboardEnteriesSize, shuffleboardEnteriesSize)
            .getEntry());

    _shuffleboardEnteries.put(
        "velocity",
        Shuffleboard.getTab(constants.subsystemName)
            .add("Velocity", 0)
            .withWidget("Graph")
            .withSize(shuffleboardEnteriesSize, shuffleboardEnteriesSize)
            .getEntry());

    _shuffleboardEnteries.put(
        "output",
        Shuffleboard.getTab(constants.subsystemName)
            .add("Output", 0)
            .withWidget("Graph")
            .withSize(shuffleboardEnteriesSize, shuffleboardEnteriesSize)
            .getEntry());

    _shuffleboardEnteries.put(
        "goalPos",
        Shuffleboard.getTab(constants.subsystemName)
            .add("Goal Position", 0)
            .withWidget("Number Bar")
            .withSize(shuffleboardEnteriesSize / 2, shuffleboardEnteriesSize)
            .getEntry());

    _shuffleboardEnteries.put(
        "goalVel",
        Shuffleboard.getTab(constants.subsystemName)
            .add("Goal Velocity", 0)
            .withWidget("Number Bar")
            .withSize(shuffleboardEnteriesSize / 2, shuffleboardEnteriesSize)
            .getEntry());

    _shuffleboardEnteries.put(
        "controlState",
        Shuffleboard.getTab(constants.subsystemName)
            .add("Control State", 0)
            .withWidget("Text View")
            .withSize(shuffleboardEnteriesSize, shuffleboardEnteriesSize / 2)
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
  public void setPercent(double percent) {
    _controlState = ControlState.PERCENT_OUTPUT;
  }

  /**
   * Sets position setpoint to the controller
   *
   * @param position - the wanted position of the controller according to the encoder
   * @see #setPercent(double)
   * @see #setVelocity(double)
   * @see #stop()
   */
  public void setPosition(double position) {
    _controlState = ControlState.PIDF_POSITION;
    _profileGoal = new State(position, 0);

    _trapozoidTimer.restart();
  }

  /**
   * Sets velocity setpoint output to the controller
   *
   * @param velocity - the wanted velocity of the controller according to the encoder
   * @see #setPercent(double)
   * @see #setPosition(double)
   * @see #stop()
   */
  public void setVelocity(double velocity) {
    _controlState = ControlState.PIDF_VELOCITY;
    _profileGoal = new State(0, velocity);

    _trapozoidTimer.restart();
  }

  /**
   * Stops the controller of all movement
   *
   * @see #setPercent(double)
   * @see #setPosition(double)
   * @see #setVelocity(double)
   */
  public void stop() {
    setPercent(0);
  }

  /**
   * @return the position and velocity of the controller
   */
  public State getEncoder() {
    return new State(_encoder.getPosition(), _encoder.getVelocity());
  }

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

  /** Resets the encoder, sets it to the home position */
  public void resetEncoder() {
    setEncoder(0);
  }

    /**
     * @return the goal/setpoint/reference of the controller, the target of PIDF / PID / Motion Magic...
     */
    public double getGoal(){
        return _demand;
    }

  /**
   * @return wheter or not the controller is at the goal, the target of PIDF / PID / Motion Magic...
   *     Will return false if not in position or velocity control
   */
  public abstract boolean atGoal();

    /**
     * Updates the shuffleboard values
     */
    protected void updateShuffleboard(){
        _shuffleboardEnteries.get("position").setDouble(getPosition());
        _shuffleboardEnteries.get("velocity").setDouble(getVelocity());
        _shuffleboardEnteries.get("output").setDouble(getOutput());
        _shuffleboardEnteries.get("goalPos").setDouble(_demand);
        _shuffleboardEnteries.get("goalVel").setDouble(_demand);
        _shuffleboardEnteries.get("controlState").setString(_controlState.name());
    }
    protected void writeControlInputs(ControlState _newState){

    }
    /**
     * Runs controller periodic tasks, run it on the subsystem periodic
     */
    public void periodic() {
        updateShuffleboard();
    }
}
