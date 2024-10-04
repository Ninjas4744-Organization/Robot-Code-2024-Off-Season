package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.StateMachineSubsystem;
import frc.robot.DataClasses.StateEndCondition;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;
import frc.robot.Swerve.SwerveIO;

import java.util.HashMap;
import java.util.Map;

public class StateMachine extends StateMachineSubsystem {
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
	 * Sets the state of the robot to the given state only if possible. For example if the current
	 * state is AMP_OUTAKE_READY it cannot change to PREPARE_AMP_OUTAKE
	 *
	 * @param wantedState - the state to change the robot state to
	 */
	public void changeRobotState(RobotStates wantedState) {
		switch (RobotState.getRobotState()) {
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

			case CLOSE, RESET:
				if (wantedState == RobotStates.IDLE) RobotState.setRobotState(wantedState);
				break;

			case IDLE:
				if (wantedState == RobotStates.NOTE_IN_INDEXER
						|| wantedState == RobotStates.NOTE_SEARCH
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE)
					RobotState.setRobotState(wantedState);
				break;

			case INTAKE, SHOOT:
				if (wantedState == RobotStates.RESET || wantedState == RobotStates.CLOSE)
					RobotState.setRobotState(wantedState);
				break;

			case CLIMB:
				if (wantedState == RobotStates.CLIMBED)
					RobotState.setRobotState(wantedState);
				break;

			case CLIMBED:
				if (wantedState == RobotStates.CLIMB_PREPARE)
					RobotState.setRobotState(wantedState);
				break;

			case CLIMB_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.CLIMB_READY) RobotState.setRobotState(wantedState);
				break;

			case SHOOT_SPEAKER_PREPARE, SHOOT_AMP_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_READY) RobotState.setRobotState(wantedState);
				break;

			case NOTE_SEARCH:
				if (wantedState == RobotStates.INTAKE ||
					wantedState == RobotStates.CLIMB_PREPARE ||
					wantedState == RobotStates.CLOSE ||
					wantedState == RobotStates.RESET)
					RobotState.setRobotState(wantedState);
				break;

			case NOTE_IN_INDEXER:
				if(wantedState == RobotStates.DRIVE_TO_AMP ||
					wantedState == RobotStates.CLIMB_PREPARE ||
					wantedState == RobotStates.SHOOT_AMP_PREPARE ||
					wantedState == RobotStates.SHOOT_SPEAKER_PREPARE ||
					wantedState == RobotStates.CLOSE ||
					wantedState == RobotStates.RESET)
					RobotState.setRobotState(wantedState);
				break;

			case DRIVE_TO_AMP:
				if (wantedState == RobotStates.SHOOT_AMP_PREPARE || wantedState == RobotStates.CLOSE)
					RobotState.setRobotState(wantedState);
				break;
		}

		if(RobotState.getRobotState() == RobotStates.IDLE)
			RobotState.setRobotState(RobotState.getNoteInIndexer() ? RobotStates.NOTE_IN_INDEXER : RobotStates.NOTE_SEARCH);
	}

	private Timer _shootTimer = new Timer();

	/**
	 * Set in this function the end condition for each state with _endConditionMap
	 */
	private void setEndConditionMap(){
		_endConditionMap.put(RobotStates.INTAKE,
			new StateEndCondition(RobotState::getNoteInIndexer, RobotStates.NOTE_IN_INDEXER));

		_endConditionMap.put(RobotStates.SHOOT_AMP_PREPARE,
			new StateEndCondition(() -> ShooterAngle.getInstance().atGoal() && Shooter.getInstance().atGoal(), RobotStates.SHOOT_READY));

		_endConditionMap.put(RobotStates.SHOOT_SPEAKER_PREPARE,
			new StateEndCondition(() -> ShooterAngle.getInstance().atGoal() && Shooter.getInstance().atGoal(), RobotStates.SHOOT_READY));

		_endConditionMap.put(RobotStates.SHOOT,
			new StateEndCondition(() -> _shootTimer.get() > 1, RobotStates.CLOSE));

		_endConditionMap.put(RobotStates.CLIMB_PREPARE,
			new StateEndCondition(() -> Climber.getInstance().atGoal(), RobotStates.CLIMB_READY));

		_endConditionMap.put(RobotStates.CLIMB,
			new StateEndCondition(() -> Climber.getInstance().atGoal(), RobotStates.CLIMBED));

		_endConditionMap.put(RobotStates.DRIVE_TO_AMP,
			new StateEndCondition(() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.SHOOT_AMP_PREPARE));

		_endConditionMap.put(RobotStates.DRIVE_TO_SOURCE,
			new StateEndCondition(() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.NOTE_SEARCH));
	}

	@Override
	public void periodic() {
		super.periodic();

		if (_endConditionMap.get(RobotState.getRobotState()).condition.getAsBoolean())
			changeRobotState(_endConditionMap.get(RobotState.getRobotState()).nextState);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(() -> _shootTimer.restart(), RobotStates.SHOOT);
	}
}
