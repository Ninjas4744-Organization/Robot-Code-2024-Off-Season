package frc.robot.AbstractClasses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import java.util.HashMap;
import java.util.Map;

public abstract class NinjasSubsystem extends SubsystemBase {
  protected NinjasController _controller;
  protected Map<RobotStates, Runnable> _functionMap;

  public NinjasSubsystem() {
    _functionMap = new HashMap<>();

    for (RobotStates state : RobotStates.values()) _functionMap.put(state, () -> {});

    setFunctionMap();
  }

  /**
   * Set in what state what function to run.
   *
   * <p>for example: _functionMap.put(RobotStates.INTAKE, () -> { System.out.println("Intaking");
   * });
   *
   * <p>doing that will spam Intaking in the console when the robot is at INTAKE state.
   */
  protected abstract void setFunctionMap();

  /**
   * Resets the subsystem: moves the subsystem down until limit hit and then stops.
   *
   * @return the command that does that
   */
  public Command resetSubsystem() {
    return runMotor(-0.5).until(() -> _controller.getPosition() <= 0);
  }

  /**
   * Runs the motor at the given percent.
   *
   * @param percent - how much to power the motor between -1 and 1
   * @return a command that runs that on start and stops to motor on end
   */
  public Command runMotor(double percent) {
    return Commands.startEnd(() -> _controller.setPercent(percent), () -> _controller.stop(), this);
  }

  @Override
  public void periodic() {
    _functionMap.get(RobotState.getRobotState()).run();
    _controller.periodic();
  }
}
