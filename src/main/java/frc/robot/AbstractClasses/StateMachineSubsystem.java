package frc.robot.AbstractClasses;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import java.util.HashMap;
import java.util.Map;

public abstract class StateMachineSubsystem extends SubsystemBase {
	private Map<RobotStates, Runnable> _periodicFunctionMap;
	private Map<RobotStates, Runnable> _onChangeFunctionMap;
	private RobotStates previousRobotState;

	public StateMachineSubsystem() {
		_periodicFunctionMap = new HashMap<>();
		_onChangeFunctionMap = new HashMap<>();

		previousRobotState = RobotState.getRobotState();

		for (RobotStates state : RobotStates.values()) _periodicFunctionMap.put(state, () -> {});
		for (RobotStates state : RobotStates.values()) _onChangeFunctionMap.put(state, () -> {});

		setFunctionMaps();
	}

	/**
	 * Set in what state what function to run.
	 *
	 * <p>There is _periodicFunctionMap that runs your function periodically every 20ms if the robot
	 * state is what you've chosen.
	 *
	 * <p>And there is also _onChangeFunctionMap that runs your function once on the moment the robot
	 * state changed to what you've chosen.
	 *
	 * <p>Examples:
	 *
	 * <p>addFunctionToOnChangeMap.put(() -> System.out.println("Started Intaking"),
	 * RobotStates.INTAKE);
	 *
	 * <p>addFunctionToPeriodicMap.put(() -> System.out.println("Intaking"), RobotStates.INTAKE);
	 *
	 * <p>Doing that will spam "Intaking" in the console when the robot is at INTAKE state and print
	 * "Started Intaking" at the moment the state changed to INTAKING.
	 *
	 * <p>Note: _onChangeFunctionMap functions always run before _periodicFunctionMap functions
	 *
	 * @see #addFunctionToOnChangeMap
	 * @see #addFunctionToPeriodicMap
	 */
	protected abstract void setFunctionMaps();

	/**
	 * adds a function to the function periodic map
	 * this function is being called periodically
	 * ATTENTION!!!
	 * only use for methods you want to be called more then once
	 *
	 * @param function - the function to add
	 * @param states - the states that the function will run at
	 * @see #setFunctionMaps
	 */
	protected void addFunctionToPeriodicMap(Runnable function, RobotStates... states) {
		for (RobotStates state : states) _periodicFunctionMap.put(state, function);
	}

	/**
	 * adds a function to the function on change map
	 * this function is being called once uppon detected
	 * change of current state to any of given states
	 *
	 * @param function - the function to add
	 * @param states - the states that the function will run at
	 * @see #setFunctionMaps
	 */
	protected void addFunctionToOnChangeMap(Runnable function, RobotStates... states) {
		for (RobotStates state : states) _onChangeFunctionMap.put(state, function);
	}

	@Override
	public void periodic() {
		if (RobotState.getRobotState() != previousRobotState)
			_onChangeFunctionMap.get(RobotState.getRobotState()).run();
		previousRobotState = RobotState.getRobotState();

		_periodicFunctionMap.get(RobotState.getRobotState()).run();
	}
}
