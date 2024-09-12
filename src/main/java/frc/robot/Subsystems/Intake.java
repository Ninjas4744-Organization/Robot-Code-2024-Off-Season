package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants.RollersConstants;
import frc.robot.RobotState.RobotStates;
import frc.robot.StateMachine;

public class Intake extends NinjasSubsystem {
	private static Intake _instance;

	private Timer _outakeTimer = new Timer();

	public static Intake getInstance() {
		if (_instance == null) _instance = new Intake();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(RollersConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(RollersConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToPeriodicMap(
				() -> {
					controller().setPercent(RollersConstants.States.kIntake);
				},
				RobotStates.INTAKE);

		addFunctionToPeriodicMap(
				() -> {
					controller().setPercent(RollersConstants.States.kOutake);

					if (_outakeTimer.get() > 1) {
						StateMachine.getInstance().changeRobotState(RobotStates.CLOSE);
					}
				},
				RobotStates.OUTAKE);

		addFunctionToOnChangeMap(() -> _outakeTimer.restart(), RobotStates.OUTAKE);
		addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.CLOSE);
		addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.RESET);
	}
}
