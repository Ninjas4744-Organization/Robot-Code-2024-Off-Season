package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DataClasses.StateEndCondition;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Rotation;

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

		for (RobotStates state : RobotStates.values()) _endConditionMap.put(state, new StateEndCondition(() -> false, RobotStates.IDLE));

		setEndConditionMap();
	}

	/**
	 * Set in this function the end condition for each state with _endConditionMap
	 */
	private void setEndConditionMap(){
		_endConditionMap.put(RobotStates.PREPARE_AMP_OUTAKE,
			new StateEndCondition(() -> Elevator.getInstance().atGoal() && Rotation.getInstance().atGoal(), RobotStates.AMP_OUTAKE_READY));
	}

	/**
	 * Sets the state of the robot to the given state only if possible. For example if the current
	 * state is AMP_OUTAKE_READY it cannot change to PREPARE_AMP_OUTAKE
	 *
	 * @param wantedState - the state to change the robot state to
	 */
	public void changeRobotState(RobotStates wantedState) {
		switch (RobotState.getRobotState()) {
			case ELEVATOR_AMP_READY, ELEVATOR_TRAP_READY:
				if (wantedState == RobotStates.ELEVATOR_OUTAKE
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

			case ELEVATOR_AMP_READY:
				if (wantedState == RobotStates.INTAKE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case CLOSE, RESET:
				if (wantedState == RobotStates.IDLE) RobotState.setRobotState(wantedState);
				break;

			case IDLE:
				if (wantedState == RobotStates.NOTE_IN_ELEVATOR
						|| wantedState == RobotStates.NOTE_SEARCH
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.PREPARE_CLIMB
						|| wantedState == RobotStates.PREPARE_SHOOT)) RobotState.setRobotState(wantedState);
				break;

			case NOTE_IN_ELEVATOR:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.ELEVATOR_TRAP_PREPARE
						|| wantedState == RobotStates.ELEVATOR_AMP_PREPARE
						|| wantedState == RobotStates.PREPARE_CLIMB
						|| wantedState == RobotStates.PREPARE_SHOOT) RobotState.setRobotState(wantedState);
				break;

			case INTAKE, SHOOT, ELEVATOR_OUTAKE:
				if (wantedState == RobotStates.RESET || wantedState == RobotStates.CLOSE)
					RobotState.setRobotState(wantedState);
				break;

			case CLIMB:
				break;

			case CLIMBED:
				if (wantedState == RobotStates.PREPARE_CLIMB)
					RobotState.setRobotState(wantedState);
				break;

			case ELEVATOR_AMP_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.ELEVATOR_AMP_READY) RobotState.setRobotState(wantedState);
				break;

			case ELEVATOR_TRAP_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.ELEVATOR_TRAP_READY) RobotState.setRobotState(wantedState);
				break;

			case PREPARE_CLIMB:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.CLIMB_READY) RobotState.setRobotState(wantedState);
				break;

			case SHOOT_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_READY) RobotState.setRobotState(wantedState);
				break;

			case NOTE_SEARCH:
				if (wantedState == RobotStates.INTAKE || wantedState == RobotStates.PREPARE_CLIMB)
					RobotState.setRobotState(wantedState);
				break;
			case NOTE_IN_ELEVATOR:
				if(wantedState == RobotStates.ELEVATOR_AMP_PREPARE || wantedState == RobotStates.ELEVATOR_TRAP_PREPARE ||wantedState == RobotStates.SHOOT_PREPARE)
					RobotState.setRobotState(wantedState);
				break;
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
						case ELEVATOR_AMP_READY,ELEVATOR_TRAP_READY:
							changeRobotState(RobotStates.ELEVATOR_OUTAKE);
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
		for(RobotStates state : RobotStates.values()){
			if(_endConditionMap.get(state).condition.getAsBoolean())
				changeRobotState(_endConditionMap.get(state).nextState);
		}
	}
}
