package frc.robot.Subsystems;

import frc.robot.NinjasLib.Subsystems.StateMachineMotoredSubsystem;

public class Climber extends StateMachineMotoredSubsystem {
	private static Climber _instance;

	public static Climber getInstance() {
		if (_instance == null) _instance = new Climber();

		return _instance;
	}

	@Override
	protected void setController() {
		//		_controller = new NinjasSparkMaxController(ClimberConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		//		_simulatedController = new NinjasSimulatedController(ClimberConstants.kSimulatedControllerConstants);
	}

	@Override
	public void resetSubsystem() {}

	@Override
	public boolean isResetted() {
		return true;
	}

	@Override
	public boolean atGoal() {
		return true;
	}

	@Override
	protected void setFunctionMaps() {
		//		addFunctionToOnChangeMap(
		//				() -> controller().setPosition(ClimberConstants.States.kUp), RobotStates.CLIMB_PREPARE);
		//
		//		addFunctionToOnChangeMap(
		//				() -> controller().setPosition(ClimberConstants.States.kClose), RobotStates.CLOSE, RobotStates.CLIMB);
		//
		//		addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
	}

	@Override
	public void periodic() {}
}
