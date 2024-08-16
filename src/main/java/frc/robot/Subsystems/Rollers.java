package frc.robot.Subsystems;

import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.DataClasses.MainControllerConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import frc.robot.StateMachine;

public class Rollers extends NinjasSubsystem {
  public Rollers() {
    super();

    _controller = new NinjasSparkMaxController(new MainControllerConstants());
  }

  @Override
  protected void setFunctionMap() {
    _functionMap.put(
        RobotStates.INTAKE,
        () -> {
          _controller.setPercent(-1);

          if (RobotState.hasNote()) {
            StateMachine.getInstance().changeRobotState(RobotStates.HOLDING_NOTE);
            _controller.stop();
          }
        });

    _functionMap.put(
        RobotStates.OUTAKE,
        () -> {
          _controller.setPercent(1);

          if (!RobotState.hasNote()) {
            StateMachine.getInstance().changeRobotState(RobotStates.NOTE_SEARCH);
            _controller.stop();
          }
        });
  }
}
