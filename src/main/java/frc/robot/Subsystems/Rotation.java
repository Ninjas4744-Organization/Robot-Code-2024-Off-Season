package frc.robot.Subsystems;

import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants.RotationConstants;
import frc.robot.RobotState.RobotStates;
import frc.robot.StateMachine;

public class Rotation extends NinjasSubsystem {
  private static Rotation _instance;

  public static Rotation getInstance() {
    if (_instance == null) _instance = new Rotation();

    return _instance;
  }

  private Rotation() {
    super();

    _controller = new NinjasSparkMaxController(RotationConstants.kControllerConstants);
  }

  @Override
  protected void setFunctionMaps() {
    addFunctionToOnChangeMap(
        () -> _controller.setPosition(RotationConstants.States.kAmp),
        RobotStates.PREPARE_AMP_OUTAKE);
    addFunctionToOnChangeMap(
        () -> _controller.setPosition(RotationConstants.States.kTrap),
        RobotStates.PREPARE_TRAP_OUTAKE);

    addFunctionToPeriodicMap(
        () -> {
          if (_controller.atGoal())
            StateMachine.getInstance().changeRobotState(RobotStates.AMP_OUTAKE_READY);
        },
        RobotStates.PREPARE_AMP_OUTAKE);
    addFunctionToPeriodicMap(
        () -> {
          if (_controller.atGoal())
            StateMachine.getInstance().changeRobotState(RobotStates.TRAP_OUTAKE_READY);
        },
        RobotStates.PREPARE_TRAP_OUTAKE);

    addFunctionToOnChangeMap(
        () -> _controller.setPosition(RotationConstants.States.kClose), RobotStates.CLOSE);
    addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotStates.RESET);
  }
}
