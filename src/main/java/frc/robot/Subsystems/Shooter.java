package frc.robot.Subsystems;

import frc.robot.NinjasLib.NinjasSimulatedController;
import frc.robot.NinjasLib.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState.RobotStates;

public class Shooter extends StateMachineMotoredSubsystem {
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
				() -> controller().setVelocity(ShooterConstants.States.kShoot), RobotStates.SHOOT);

		addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.CLOSE);
	}
}
