package frc.robot.Subsystems;

import frc.robot.NinjasLib.NinjasSimulatedController;
import frc.robot.NinjasLib.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.RobotState;

public class ShooterFeeder extends StateMachineMotoredSubsystem {
	private static ShooterFeeder _instance;

	public static ShooterFeeder getInstance() {
		if (_instance == null) _instance = new ShooterFeeder();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ShooterFeederConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(ShooterFeederConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
				() -> controller().setPercent(ShooterFeederConstants.States.kRoll),
				RobotState.RobotStates.SHOOT);

		addFunctionToOnChangeMap(
				() -> controller().stop(),
				RobotState.RobotStates.CLOSE);
	}
}
