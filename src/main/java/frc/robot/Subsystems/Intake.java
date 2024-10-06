package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.NinjasLib.NinjasSimulatedController;
import frc.robot.NinjasLib.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;

public class Intake extends StateMachineMotoredSubsystem {
	private static Intake _instance;

	private Timer _outakeTimer = new Timer();

	public static Intake getInstance() {
		if (_instance == null) _instance = new Intake();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(IntakeConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(IntakeConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
				() -> controller().setPercent(IntakeConstants.States.kIntake),
				RobotStates.INTAKE);

		addFunctionToOnChangeMap(
				() -> controller().stop(),
				RobotState.RobotStates.CLOSE);

		addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotStates.RESET);

	}
}
