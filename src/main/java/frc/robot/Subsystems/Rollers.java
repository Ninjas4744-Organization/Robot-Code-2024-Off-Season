package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.RollersConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import frc.robot.StateMachine;

public class Rollers extends NinjasSubsystem {
  private static Rollers _instance;
  private Timer _outakeTimer = new Timer();

  public static Rollers getInstance() {
    if (_instance == null) _instance = new Rollers();

    return _instance;
  }

  private Rollers() {
    super();

    _controller = new NinjasSparkMaxController(RollersConstants.kControllerConstants);
  }

  @Override
  protected void setFunctionMaps() {
    addFunctionToPeriodicMap(
        () -> {
          _controller.setPosition(Constants.RotationConstants.States.kAmp);

          if (RobotState.hasNote()) {
            StateMachine.getInstance().changeRobotState(RobotStates.CLOSE);
          }
        },
        RobotStates.INTAKE);

    addFunctionToPeriodicMap(
        () -> {
          _controller.setPercent(1);

          if (_outakeTimer.get() > 1) {
            StateMachine.getInstance().changeRobotState(RobotStates.CLOSE);
          }
        },
        RobotStates.OUTAKE);

    addFunctionToOnChangeMap(() -> _outakeTimer.restart(), RobotStates.OUTAKE);
    addFunctionToOnChangeMap(() -> _controller.stop(), RobotStates.CLOSE);
    addFunctionToOnChangeMap(() -> _controller.stop(), RobotStates.RESET);
  }
}
