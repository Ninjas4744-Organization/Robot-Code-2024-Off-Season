package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.StateMachineMotoredSubsystem;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.RobotState;
import frc.robot.StateMachine;

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
