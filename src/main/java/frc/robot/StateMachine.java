package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.NinjasLib.DataClasses.StateEndCondition;
import frc.robot.NinjasLib.StateMachineIO;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ShooterAngle;
import frc.robot.Swerve.SwerveIO;

public class StateMachine extends StateMachineIO<RobotStates> {
	public static StateMachine getInstance() {
		return (StateMachine) StateMachineIO.getInstance();
	}

	@Override
	public void changeRobotState(RobotStates wantedState) {
		switch (RobotState.getInstance().getRobotState()) {
			case SHOOT_SPEAKER_READY:
				if (wantedState == RobotStates.SHOOT
						|| wantedState == RobotStates.SHOOT_SPEAKER_PREPARE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
				break;

			case SHOOT_AMP_READY:
				if (wantedState == RobotStates.SHOOT
						|| wantedState == RobotStates.SHOOT_AMP_PREPARE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
				break;

			case CLIMB_READY:
				if (wantedState == RobotStates.CLIMB
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
				break;

			case CLOSE, RESET:
				if (wantedState == RobotStates.IDLE) RobotState.getInstance().setRobotState(wantedState);
				break;

			case IDLE:
				if (wantedState == RobotStates.NOTE_IN_INDEXER
						|| wantedState == RobotStates.NOTE_SEARCH
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE) RobotState.getInstance().setRobotState(wantedState);
				break;

			case SHOOT:
				if (wantedState == RobotStates.RESET || wantedState == RobotStates.CLOSE)
					RobotState.getInstance().setRobotState(wantedState);
				break;

			case INTAKE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.NOTE_IN_INDEXER
						|| wantedState == RobotStates.INDEX
						|| wantedState == RobotStates.OUTTAKE) RobotState.getInstance().setRobotState(wantedState);
				break;

			case CLIMB:
				if (wantedState == RobotStates.CLIMBED) RobotState.getInstance().setRobotState(wantedState);
				break;

			case CLIMBED:
				if (wantedState == RobotStates.CLIMB_PREPARE) RobotState.getInstance().setRobotState(wantedState);
				break;

			case CLIMB_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.CLIMB_READY) RobotState.getInstance().setRobotState(wantedState);
				break;

			case SHOOT_SPEAKER_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_SPEAKER_READY) RobotState.getInstance().setRobotState(wantedState);
				break;

			case SHOOT_AMP_PREPARE:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_AMP_READY) RobotState.getInstance().setRobotState(wantedState);
				break;

			case NOTE_SEARCH:
				if (wantedState == RobotStates.INTAKE
						|| wantedState == RobotStates.CLIMB_PREPARE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.SHOOT_SPEAKER_PREPARE
					|| wantedState == RobotStates.OUTTAKE
						|| wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
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
					|| wantedState == RobotStates.OOGA_BOOGA) RobotState.getInstance().setRobotState(wantedState);
				break;

			case DRIVE_TO_AMP:
				if (wantedState == RobotStates.SHOOT_AMP_PREPARE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
				break;

			case DRIVE_TO_SOURCE:
				if (wantedState == RobotStates.INTAKE
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
				break;

			case INDEX:
				if (wantedState == RobotStates.INDEX_BACK
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.OUTTAKE) RobotState.getInstance().setRobotState(wantedState);
				break;

			case INDEX_BACK:
				if (wantedState == RobotStates.NOTE_IN_INDEXER
						|| wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.OUTTAKE) RobotState.getInstance().setRobotState(wantedState);
				break;

			case TESTING:
				if (wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
				break;

			case OUTTAKE:
				if (wantedState == RobotStates.CLOSE || wantedState == RobotStates.RESET)
					RobotState.getInstance().setRobotState(wantedState);
				break;

			case DELIVERY:
				if (wantedState == RobotStates.CLOSE
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.SHOOT) RobotState.getInstance().setRobotState(wantedState);
				break;

			case OOGA_BOOGA:
				if (wantedState == RobotStates.OOGA_BOOGA_READY
					|| wantedState == RobotStates.CLOSE
					|| wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
				break;

			case OOGA_BOOGA_READY:
				if (wantedState == RobotStates.OOGA_BOOGA
					|| wantedState == RobotStates.CLOSE
					|| wantedState == RobotStates.SHOOT
					|| wantedState == RobotStates.RESET) RobotState.getInstance().setRobotState(wantedState);
				break;
		}

		if (RobotState.getInstance().getRobotState() == RobotStates.IDLE)
			RobotState.getInstance().setRobotState(
					RobotState.getInstance().getNoteInIndexer() ? RobotStates.NOTE_IN_INDEXER : RobotStates.NOTE_SEARCH);
	}

	private final Timer _shootTimer = new Timer();
	private final Timer _outtakeTimer = new Timer();

	@Override
	protected void setEndConditionMap() {
		addEndCondition(
				RobotStates.RESET,
				new StateEndCondition<>(
						() -> ShooterAngle.getInstance().isResetted()
								&& Indexer.getInstance().isResetted()
								&& Shooter.getInstance().isResetted(),
						RobotStates.IDLE));

		addEndCondition(
				RobotStates.CLOSE,
				new StateEndCondition<>(
						() -> ShooterAngle.getInstance().isHomed()
								&& Indexer.getInstance().isResetted()
								&& Shooter.getInstance().isResetted(),
						RobotStates.IDLE));

		addEndCondition(
				RobotStates.INTAKE, new StateEndCondition<>(RobotState.getInstance()::getNoteInIndexer, RobotStates.INDEX));

		addEndCondition(
				RobotStates.INDEX, new StateEndCondition<>(() -> !RobotState.getInstance().getNoteInIndexer(), RobotStates.INDEX_BACK));

		addEndCondition(
				RobotStates.INDEX_BACK,
				new StateEndCondition<>(RobotState.getInstance()::getNoteInIndexer, RobotStates.NOTE_IN_INDEXER));

		addEndCondition(
				RobotStates.OUTTAKE, new StateEndCondition<>(() -> _outtakeTimer.get() > 1, RobotStates.CLOSE));

		addEndCondition(
				RobotStates.SHOOT_AMP_PREPARE,
				new StateEndCondition<>(
						() -> ShooterAngle.getInstance().atGoal()
								&& Shooter.getInstance().isReady(),
						RobotStates.SHOOT_AMP_READY));

		addEndCondition(
				RobotStates.SHOOT_SPEAKER_PREPARE,
				new StateEndCondition<>(
						() -> ShooterAngle.getInstance().atGoal()
								&& Shooter.getInstance().isReady(),
						RobotStates.SHOOT_SPEAKER_READY));

		addEndCondition(
			RobotStates.OOGA_BOOGA,
			new StateEndCondition<>(
				() -> ShooterAngle.getInstance().atGoal()
					&& Shooter.getInstance().isReady(),
				RobotStates.OOGA_BOOGA_READY));

		addEndCondition(
			RobotStates.OOGA_BOOGA_READY,
			new StateEndCondition<>(
				() -> ShooterAngle.getInstance().atGoal()
					&& Shooter.getInstance().isReady(),
				RobotStates.OOGA_BOOGA));

		addEndCondition(
				RobotStates.DELIVERY,
				new StateEndCondition<>(
						() -> ShooterAngle.getInstance().atGoal()
								&& Shooter.getInstance().isReady(),
						RobotStates.SHOOT));

		addEndCondition(RobotStates.SHOOT_AMP_READY, new StateEndCondition<>(() -> true, RobotStates.SHOOT));

		addEndCondition(
				RobotStates.SHOOT_SPEAKER_READY,
				new StateEndCondition<>(
						() -> !ShooterAngle.getInstance().atGoal()
								|| !Shooter.getInstance().isReady(),
						RobotStates.SHOOT_SPEAKER_PREPARE));

		addEndCondition(RobotStates.SHOOT, new StateEndCondition<>(() -> _shootTimer.get() > 1, RobotStates.CLOSE));

		addEndCondition(
				RobotStates.DRIVE_TO_AMP,
				new StateEndCondition<>(
						() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.SHOOT_AMP_PREPARE));

		addEndCondition(
				RobotStates.DRIVE_TO_SOURCE,
				new StateEndCondition<>(() -> SwerveIO.getInstance().isPathFollowingFinished(), RobotStates.INTAKE));

		addEndCondition(RobotStates.IDLE, new StateEndCondition<>(() -> RobotState.getInstance().getNoteInIndexer(), RobotStates.NOTE_IN_INDEXER));
		addEndCondition(RobotStates.IDLE, new StateEndCondition<>(() -> !RobotState.getInstance().getNoteInIndexer(), RobotStates.NOTE_SEARCH));
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(_shootTimer::restart, RobotStates.SHOOT);
		addFunctionToOnChangeMap(_outtakeTimer::restart, RobotStates.OUTTAKE);
	}
}
