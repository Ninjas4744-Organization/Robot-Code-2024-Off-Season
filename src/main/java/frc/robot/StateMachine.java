package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NinjasLib.DataClasses.StateEndCondition;
import frc.robot.NinjasLib.Subsystems.StateMachineSubsystem;
import frc.robot.NinjasLib.Swerve.SwerveIO;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;

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
			_endConditionMap.put(state, new StateEndCondition(() -> false, state));

		setEndConditionMap();
	}

	public void setTriggerForSimulationTesting(Trigger trigger) {
		if (RobotState.isSimulated())
			trigger.onTrue(Commands.runOnce(
					() -> changeRobotState(_endConditionMap.get(RobotState.getRobotState()).nextState)));
	}

	/**
	 * Sets the state of the robot to the given state only if possible. For example if the current
	 * state is AMP_OUTAKE_READY it cannot change to PREPARE_AMP_OUTAKE
	 *
	 * @param wantedState - the state to change the robot state to
	 */
	public void changeRobotState(RobotStates wantedState) {
		switch (RobotState.getRobotState()) {
			case SHOOT_SPEAKER_READY:
				if (wantedState == RobotStates.SHOOT
						|| wantedState == RobotStates.SHOOT_SPEAKER_PREPARE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case SHOOT_AMP_READY:
				if (wantedState == RobotStates.SHOOT
						|| wantedState == RobotStates.SHOOT_AMP_PREPARE
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
						|| wantedState == RobotStates.CLOSE) RobotState.setRobotState(wantedState);
				break;

			case SHOOT:
				if (wantedState == RobotStates.RESET || wantedState == RobotStates.CLOSE)
					RobotState.setRobotState(wantedState);
				break;

			case INTAKE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.NOTE_IN_INDEXER
						|| wantedState == RobotStates.INDEX
						|| wantedState == RobotStates.OUTTAKE) RobotState.setRobotState(wantedState);
				break;

			case CLIMB:
				if (wantedState == RobotStates.CLIMBED) RobotState.setRobotState(wantedState);
				break;

			case CLIMBED:
				if (wantedState == RobotStates.CLIMB_PREPARE) RobotState.setRobotState(wantedState);
				break;

			case CLIMB_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.CLIMB_READY) RobotState.setRobotState(wantedState);
				break;

			case SHOOT_SPEAKER_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_SPEAKER_READY) RobotState.setRobotState(wantedState);
				break;

			case SHOOT_AMP_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_AMP_READY) RobotState.setRobotState(wantedState);
				break;

			case NOTE_SEARCH:
				if (wantedState == RobotStates.INTAKE
						|| wantedState == RobotStates.CLIMB_PREPARE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_SPEAKER_PREPARE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case NOTE_IN_INDEXER:
				if (wantedState == RobotStates.DRIVE_TO_AMP
						|| wantedState == RobotStates.DRIVE_TO_SOURCE
						|| wantedState == RobotStates.CLIMB_PREPARE
						|| wantedState == RobotStates.SHOOT_AMP_PREPARE
						|| wantedState == RobotStates.SHOOT_SPEAKER_PREPARE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.DELIVERY
						|| wantedState == RobotStates.RESET
					|| wantedState == RobotStates.OUTTAKE
					|| wantedState == RobotStates.OOGA_BOOGA) RobotState.setRobotState(wantedState);
				break;

			case DRIVE_TO_AMP:
				if (wantedState == RobotStates.SHOOT_AMP_PREPARE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case DRIVE_TO_SOURCE:
				if (wantedState == RobotStates.INTAKE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case INDEX:
				if (wantedState == RobotStates.INDEX_BACK
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.OUTTAKE) RobotState.setRobotState(wantedState);
				break;

			case INDEX_BACK:
				if (wantedState == RobotStates.NOTE_IN_INDEXER
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.OUTTAKE) RobotState.setRobotState(wantedState);
				break;

			case TESTING:
				if (wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case OUTTAKE:
				if (wantedState == RobotStates.CLOSE || wantedState == RobotStates.RESET)
					RobotState.setRobotState(wantedState);
				break;

			case DELIVERY:
				if (wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.SHOOT) RobotState.setRobotState(wantedState);
				break;

			case OOGA_BOOGA:
				if (wantedState == RobotStates.OOGA_BOOGA_READY
					|| wantedState == RobotStates.CLOSE
					|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;

			case OOGA_BOOGA_READY:
				if (wantedState == RobotStates.OOGA_BOOGA
					|| wantedState == RobotStates.CLOSE
					|| wantedState == RobotStates.SHOOT
					|| wantedState == RobotStates.RESET) RobotState.setRobotState(wantedState);
				break;
		}

		if (RobotState.getRobotState() == RobotStates.IDLE)
			RobotState.setRobotState(
					RobotState.getNoteInIndexer() ? RobotStates.NOTE_IN_INDEXER : RobotStates.NOTE_SEARCH);
	}

	private Timer _shootTimer = new Timer();
	private Timer _outtakeTimer = new Timer();

	/**
	 * Set in this function the end condition for each state with _endConditionMap
	 */
	private void setEndConditionMap() {
		_endConditionMap.put(
				RobotStates.RESET,
				new StateEndCondition(
						() -> ShooterAngle.getInstance().isResetted()
								&& Indexer.getInstance().isResetted()
								&& Shooter.getInstance().isResetted(),
						RobotStates.IDLE));

		_endConditionMap.put(
				RobotStates.CLOSE,
				new StateEndCondition(
						() -> ShooterAngle.getInstance().isHomed()
								&& Indexer.getInstance().isResetted()
								&& Shooter.getInstance().isResetted(),
						RobotStates.IDLE));

		_endConditionMap.put(
				RobotStates.INTAKE, new StateEndCondition(RobotState::getNoteInIndexer, RobotStates.INDEX));

		_endConditionMap.put(
				RobotStates.INDEX, new StateEndCondition(() -> !RobotState.getNoteInIndexer(), RobotStates.INDEX_BACK));

		_endConditionMap.put(
				RobotStates.INDEX_BACK,
				new StateEndCondition(RobotState::getNoteInIndexer, RobotStates.NOTE_IN_INDEXER));

		_endConditionMap.put(
				RobotStates.OUTTAKE, new StateEndCondition(() -> _outtakeTimer.get() > 1, RobotStates.CLOSE));

		_endConditionMap.put(
				RobotStates.SHOOT_AMP_PREPARE,
				new StateEndCondition(
						() -> ShooterAngle.getInstance().atGoal()
								&& Shooter.getInstance().isReady(),
						RobotStates.SHOOT_AMP_READY));

		_endConditionMap.put(
				RobotStates.SHOOT_SPEAKER_PREPARE,
				new StateEndCondition(
						() -> ShooterAngle.getInstance().atGoal()
								&& Shooter.getInstance().isReady(),
						RobotStates.SHOOT_SPEAKER_READY));

		_endConditionMap.put(
			RobotStates.OOGA_BOOGA,
			new StateEndCondition(
				() -> ShooterAngle.getInstance().atGoal()
					&& Shooter.getInstance().isReady(),
				RobotStates.OOGA_BOOGA_READY));

		_endConditionMap.put(
			RobotStates.OOGA_BOOGA_READY,
			new StateEndCondition(
				() -> ShooterAngle.getInstance().atGoal()
					&& Shooter.getInstance().isReady(),
				RobotStates.OOGA_BOOGA));

		_endConditionMap.put(
				RobotStates.DELIVERY,
				new StateEndCondition(
						() -> ShooterAngle.getInstance().atGoal()
								&& Shooter.getInstance().isReady(),
						RobotStates.SHOOT));

		_endConditionMap.put(RobotStates.SHOOT_AMP_READY, new StateEndCondition(() -> true, RobotStates.SHOOT));

		_endConditionMap.put(
				RobotStates.SHOOT_SPEAKER_READY,
				new StateEndCondition(
						() -> !ShooterAngle.getInstance().atGoal()
								|| !Shooter.getInstance().isReady(),
						RobotStates.SHOOT_SPEAKER_PREPARE));

		_endConditionMap.put(RobotStates.SHOOT, new StateEndCondition(() -> _shootTimer.get() > 1, RobotStates.CLOSE));

		_endConditionMap.put(
				RobotStates.DRIVE_TO_AMP,
				new StateEndCondition(
						() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.SHOOT_AMP_PREPARE));

		_endConditionMap.put(
				RobotStates.DRIVE_TO_SOURCE,
				new StateEndCondition(() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.INTAKE));
	}

	@Override
	public void periodic() {
		super.periodic();

		if (_endConditionMap.get(RobotState.getRobotState()).condition.getAsBoolean() && !RobotState.isSimulated())
			changeRobotState(_endConditionMap.get(RobotState.getRobotState()).nextState);

		if (RobotState.getRobotState() == RobotStates.NOTE_IN_INDEXER && !RobotState.getNoteInIndexer())
			RobotState.setRobotState(RobotStates.NOTE_SEARCH);

		if (RobotState.getRobotState() == RobotStates.NOTE_SEARCH && RobotState.getNoteInIndexer())
			RobotState.setRobotState(RobotStates.NOTE_IN_INDEXER);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(() -> _shootTimer.restart(), RobotStates.SHOOT);
		addFunctionToOnChangeMap(() -> _outtakeTimer.restart(), RobotStates.OUTTAKE);
	}
}
