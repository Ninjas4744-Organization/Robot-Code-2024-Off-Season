package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DataClasses.StateEndCondition;
import frc.robot.RobotState.RobotStates;
import frc.robot.Swerve.SwerveIO;

import java.util.HashMap;
import java.util.Map;

public class StateMachine extends SubsystemBase {
	private static StateMachine _instance;

	public static StateMachine getInstance() {
		if (_instance == null) _instance = new StateMachine();

		return _instance;
	}

	private Map<RobotStates, StateEndCondition> _endConditionMap;

	private StateMachine() {
		_endConditionMap = new HashMap<>();

		for (RobotStates state : RobotStates.values())
			_endConditionMap.put(state, new StateEndCondition(() -> false, RobotStates.IDLE));

		setEndConditionMap();
	}

	/**
	 * Set in this function the end condition for each state with _endConditionMap
	 */
	private void setEndConditionMap() {
		_endConditionMap.put(RobotStates.DRIVE_TO_AMP, new StateEndCondition(() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.PREPARE_AMP_OUTAKE));
	}

	/**
	 * Sets the state of the robot to the given state only if possible. For example if the current
	 * state is AMP_OUTAKE_READY it cannot change to PREPARE_AMP_OUTAKE
	 *
	 * @param wantedState - the state to change the robot state to
	 */
	public void changeRobotState(RobotStates wantedState) {
		switch (RobotState.getRobotState()) {
			case AMP_OUTAKE_READY, TRAP_OUTAKE_READY:
				if (wantedState == RobotStates.OUTAKE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case SHOOT_READY:
				if (wantedState == RobotStates.SHOOT
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case CLIMB_READY:
				if (wantedState == RobotStates.CLIMB
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case INTAKE_READY:
				if (wantedState == RobotStates.INTAKE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case CLOSE, RESET:
				if (wantedState == RobotStates.IDLE) RobotState.setRobotState(wantedState);
				break;

			case IDLE:
				if (wantedState == RobotStates.HOLDING_NOTE
						|| wantedState == RobotStates.NOTE_SEARCH
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.PREPARE_CLIMB
						|| wantedState == RobotStates.PREPARE_INTAKE) RobotState.setRobotState(wantedState);
				break;

			case HOLDING_NOTE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.PREPARE_AMP_OUTAKE
						|| wantedState == RobotStates.PREPARE_TRAP_OUTAKE
						|| wantedState == RobotStates.PREPARE_CLIMB
						|| wantedState == RobotStates.PREPARE_SHOOT
						|| wantedState == RobotStates.DRIVE_TO_AMP) RobotState.setRobotState(wantedState);
				break;

			case INTAKE, OUTAKE:
				if (wantedState == RobotStates.RESET || wantedState == RobotStates.CLOSE)
					RobotState.setRobotState(wantedState);
				break;

			case SHOOT:
				if (wantedState == RobotStates.RESET || wantedState == RobotStates.IDLE)
					RobotState.setRobotState(wantedState);
				break;

			case CLIMB:
				break;

			case CLIMBED:
				if (wantedState == RobotStates.PREPARE_TRAP_OUTAKE || wantedState == RobotStates.PREPARE_CLIMB)
					RobotState.setRobotState(wantedState);
				break;

			case PREPARE_AMP_OUTAKE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.AMP_OUTAKE_READY) {
					RobotState.setRobotState(wantedState);
				}
				break;

			case PREPARE_TRAP_OUTAKE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.TRAP_OUTAKE_READY) RobotState.setRobotState(wantedState);
				break;

			case PREPARE_CLIMB:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.CLIMB_READY) RobotState.setRobotState(wantedState);
				break;

			case PREPARE_INTAKE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.INTAKE_READY) RobotState.setRobotState(wantedState);
				break;

			case PREPARE_SHOOT:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_READY) RobotState.setRobotState(wantedState);
				break;

			case NOTE_SEARCH:
				if (wantedState == RobotStates.PREPARE_INTAKE || wantedState == RobotStates.PREPARE_CLIMB)
					RobotState.setRobotState(wantedState);
				break;

			case DRIVE_TO_AMP:
				if (wantedState == RobotStates.PREPARE_AMP_OUTAKE || wantedState == RobotStates.CLOSE)
					RobotState.setRobotState(wantedState);
				break;
		}

		if (RobotState.getRobotState() == RobotStates.CLOSE) RobotState.setRobotState(RobotStates.IDLE);

		if (RobotState.getRobotState() == RobotStates.IDLE) {
			if (RobotState.hasNote())
				RobotState.setRobotState(RobotStates.HOLDING_NOTE);
			else
				RobotState.setRobotState(RobotStates.NOTE_SEARCH);
		}
	}

	/**
	 * Sets the robot state to the state according to the action that should be taken. Used for when
	 * you press a button on the controller it will change to the right state
	 *
	 * @return the command that changes the state
	 */
	public Command Act() {
		return Commands.runOnce(
				() -> {
					switch (RobotState.getRobotState()) {
						case AMP_OUTAKE_READY, TRAP_OUTAKE_READY:
							changeRobotState(RobotStates.OUTAKE);
							break;

						case CLIMB_READY:
							changeRobotState(RobotStates.CLIMB);
							break;

						case SHOOT_READY:
							changeRobotState(RobotStates.SHOOT);
							break;

						default:
							break;
					}
				},
				this);
	}

	@Override
	public void periodic() {
		if (_endConditionMap.get(RobotState.getRobotState()).condition.getAsBoolean())
			changeRobotState(_endConditionMap.get(RobotState.getRobotState()).nextState);
	}
}
