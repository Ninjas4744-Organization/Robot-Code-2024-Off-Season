package frc.robot.Subsystems;

import frc.robot.Constants.ClimberConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.RobotState.RobotStates;

public class Climber extends StateMachineMotoredSubsystem {
	private static Climber _instance;

  public Climber(boolean disabled) {
    super(disabled);
  }

  public static void disable() {
    if (_instance == null)
      _instance = new Climber(true);
  }

	public static Climber getInstance() {
    if (_instance == null) _instance = new Climber(false);

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ClimberConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(ClimberConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
				() -> controller().setPosition(ClimberConstants.States.kUp), RobotStates.CLIMB_PREPARE);

		addFunctionToOnChangeMap(
				() -> controller().setPosition(ClimberConstants.States.kClose), RobotStates.CLOSE, RobotStates.CLIMB);

		addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotStates.RESET);
	}
}
