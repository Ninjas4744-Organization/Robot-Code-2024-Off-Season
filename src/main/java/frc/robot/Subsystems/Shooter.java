package frc.robot.Subsystems;

import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState;

public class Shooter extends NinjasSubsystem {
	private static Shooter _instance;

	public static Shooter getInstance() {
		if (_instance == null) _instance = new Shooter();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ShooterConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(ShooterConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
				() -> controller().setVelocity(ShooterConstants.kShootVelocity), RobotState.RobotStates.PREPARE_SHOOT);

		addFunctionToOnChangeMap(() -> controller().stop(), RobotState.RobotStates.NOTE_SEARCH);
	}
}
