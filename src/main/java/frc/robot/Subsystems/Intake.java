package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.RollersConstants;
import frc.robot.RobotState;
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
		addFunctionToOnChangeMap(
				() -> {
					controller().setPercent(RollersConstants.States.kIntake);
				},
				RobotStates.INTAKE);
		addFunctionToOnChangeMap(
				() -> {
					controller().stop();
				},
				RobotState.RobotStates.ELEVATOR_AMP_PREPARE,
				RobotState.RobotStates.ELEVATOR_TRAP_PREPARE,
				RobotState.RobotStates.CLOSE);
		addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotStates.RESET);

	}
}
