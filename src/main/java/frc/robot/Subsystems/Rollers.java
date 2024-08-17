package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
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

  @Override
  protected void setControllers() {
    _controller = new NinjasSparkMaxController(RollersConstants.kControllerConstants);
    _simulatedController =
        new NinjasSimulatedController(RollersConstants.kSimulatedControllerConstants);
  }

  @Override
  protected void setFunctionMaps() {
    addFunctionToPeriodicMap(
        () -> {
          controller().setPercent(RollersConstants.States.kIntake);

          if (RobotState.hasNote()) StateMachine.getInstance().changeRobotState(RobotStates.CLOSE);
        },
        RobotStates.INTAKE);

    addFunctionToPeriodicMap(
        () -> {
          controller().setPercent(RollersConstants.States.kOutake);

          if (_outakeTimer.get() > 1) {
            StateMachine.getInstance().changeRobotState(RobotStates.CLOSE);
          }
        },
        RobotStates.OUTAKE);

    addFunctionToOnChangeMap(() -> _outakeTimer.restart(), RobotStates.OUTAKE);
    addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.CLOSE);
    addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.RESET);
  }
}
