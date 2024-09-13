package frc.robot.AbstractClasses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;
import java.util.HashMap;
import java.util.Map;

public abstract class NinjasSubsystem extends SubsystemBase {
	protected NinjasController _controller;
	protected NinjasSimulatedController _simulatedController;

	private final Map<RobotStates, Runnable> _periodicFunctionMap;
	private final Map<RobotStates, Runnable> _onChangeFunctionMap;
	private RobotStates previousRobotState;

	public NinjasSubsystem() {
		_periodicFunctionMap = new HashMap<>();
		_onChangeFunctionMap = new HashMap<>();

		previousRobotState = RobotState.getRobotState();

		for (RobotStates state : RobotStates.values()) _periodicFunctionMap.put(state, () -> {});
		for (RobotStates state : RobotStates.values()) _onChangeFunctionMap.put(state, () -> {});

		if (RobotState.isSimulated()) setSimulationController();
		else setController();

		setFunctionMaps();
	}

	/**
	 * Set the real controller of the subsystem.
	 *
	 * <p>Implement controller in the _controller variable,
	 */
	protected abstract void setController();

	/**
	 * Set the simulated controller of the subsystem.
	 *
	 * <p>Implement controller in the _simulatedController for the simulated one.
	 *
	 * <p>The simulated controller is optional, only set it if code will be simulated.
	 */
	protected abstract void setSimulationController();

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
	 *
	 * @param function - the function to add
	 * @param states - the states that the function will run at
	 * @see #setFunctionMaps
	 */
	protected void addFunctionToOnChangeMap(Runnable function, RobotStates... states) {
		for (RobotStates state : states) _onChangeFunctionMap.put(state, function);
	}

	protected NinjasController controller() {
		if (RobotState.isSimulated()) return _simulatedController;
		else return _controller;
	}

	/**
	 * Resets the subsystem: moves the subsystem down until limit hit and then stops.
	 *
	 * @return the command that does that
	 */
	public Command resetSubsystem() {
		return runMotor(-0.3).until(() -> controller().getPosition() <= 0);
	}

	/**
	 * @return Whether the subsystem is homed/reseted/closed
	 */
	public boolean isHomed() {
		return controller().isHomed();
	}

	/**
	 * @return Whether the subsystem is at its PIDF goal
	 */
	public boolean atGoal() {
		return controller().atGoal();
	}

	/**
	 * Runs the motor at the given percent.
	 *
	 * @param percent - how much to power the motor between -1 and 1
	 * @return a command that runs that on start and stops to motor on end
	 */
	public Command runMotor(double percent) {
		return Commands.startEnd(
				() -> controller().setPercent(percent), () -> controller().stop(), this);
	}

	@Override
	public void periodic() {
		if (RobotState.getRobotState() != previousRobotState)
			_onChangeFunctionMap.get(RobotState.getRobotState()).run();
		previousRobotState = RobotState.getRobotState();

		_periodicFunctionMap.get(RobotState.getRobotState()).run();

		controller().periodic();
	}
}
