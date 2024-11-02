package frc.robot.Subsystems;

import frc.robot.Constants.IndexerConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasTalonFXController;
import frc.robot.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.RobotStates;

public class Indexer extends StateMachineMotoredSubsystem<RobotStates> {
	private static Indexer _instance;

	public static Indexer getInstance() {
		if (_instance == null) _instance = new Indexer();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasTalonFXController(IndexerConstants.kControllerConstants);
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
				() -> controller().setPercent(IndexerConstants.States.kShoot.get()), RobotStates.SHOOT);

		addFunctionToOnChangeMap(
				() -> controller().setPercent(IndexerConstants.States.kOuttake.get()), RobotStates.OUTTAKE);

		addFunctionToOnChangeMap(
				() -> controller().stop(), RobotStates.CLOSE, RobotStates.NOTE_IN_INDEXER);

		addFunctionToOnChangeMap(
				() -> controller().setPercent(IndexerConstants.States.kIntake.get()), RobotStates.INTAKE);
		addFunctionToOnChangeMap(
				() -> controller().setPercent(IndexerConstants.States.kIndex.get()), RobotStates.INDEX);
		addFunctionToOnChangeMap(
				() -> controller().setPercent(IndexerConstants.States.kIndexBack.get()), RobotStates.INDEX_BACK);

		addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET, RobotStates.CLOSE);
	}
}
