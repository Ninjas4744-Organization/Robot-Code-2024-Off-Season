package frc.robot.Subsystems;

import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotState.RobotStates;
import frc.robot.StateMachine;

public class Elevator extends NinjasSubsystem {
	private static Elevator _instance;

	public static Elevator getInstance() {
		if (_instance == null) _instance = new Elevator();

		return _instance;
	}

	@Override
	protected void setControllers() {
		_controller = new NinjasSparkMaxController(ElevatorConstants.kControllerConstants);
		_simulatedController = new NinjasSimulatedController(ElevatorConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
				() -> controller().setPosition(ElevatorConstants.States.kAmp), RobotStates.PREPARE_AMP_OUTAKE);
		addFunctionToOnChangeMap(
				() -> controller().setPosition(ElevatorConstants.States.kTrap), RobotStates.PREPARE_TRAP_OUTAKE);

		addFunctionToPeriodicMap(
				() -> {
					if (controller().atGoal())
						StateMachine.getInstance().changeRobotState(RobotStates.AMP_OUTAKE_READY);
				},
				RobotStates.PREPARE_AMP_OUTAKE);
		addFunctionToPeriodicMap(
				() -> {
					if (controller().atGoal())
						StateMachine.getInstance().changeRobotState(RobotStates.TRAP_OUTAKE_READY);
				},
				RobotStates.PREPARE_TRAP_OUTAKE);

		addFunctionToOnChangeMap(() -> controller().setPosition(ElevatorConstants.States.kClose), RobotStates.CLOSE);
		addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotStates.RESET);
	}
}
