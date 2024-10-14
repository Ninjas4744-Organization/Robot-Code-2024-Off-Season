package frc.robot.Subsystems;

import frc.robot.Constants.IndexerConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasSparkMaxController;
import frc.robot.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.RobotState;

public class Indexer extends StateMachineMotoredSubsystem {
	private static Indexer _instance;

	public static Indexer getInstance() {
		if (_instance == null) _instance = new Indexer();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(IndexerConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(IndexerConstants.kSimulatedControllerConstants);
	}

	@Override
	public void resetSubsystem() {
		controller().stop();
	}

	@Override
	public boolean isResetted() {
		return controller().getOutput() == 0;
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
				() -> controller().setPercent(IndexerConstants.States.kShoot), RobotState.RobotStates.SHOOT);

		addFunctionToOnChangeMap(
				() -> controller().stop(), RobotState.RobotStates.CLOSE, RobotState.RobotStates.NOTE_IN_INDEXER);

		addFunctionToOnChangeMap(
				() -> controller().setPercent(IndexerConstants.States.kIntake), RobotState.RobotStates.INTAKE);
		addFunctionToOnChangeMap(
				() -> controller().setPercent(IndexerConstants.States.kIndex), RobotState.RobotStates.INDEX);
		addFunctionToOnChangeMap(
				() -> controller().setPercent(IndexerConstants.States.kIndexBack), RobotState.RobotStates.INDEX_BACK);

		addFunctionToOnChangeMap(this::resetSubsystem, RobotState.RobotStates.RESET);
	}
}
