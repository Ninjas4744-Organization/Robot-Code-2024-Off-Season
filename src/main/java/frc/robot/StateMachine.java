package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DataClasses.StateEndCondition;
import frc.robot.RobotState.RobotStates;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Rotation;

import java.util.HashMap;
import java.util.Map;

public class StateMachine extends SubsystemBase {
	private static StateMachine _instance;
	private DigitalInput _bimBreakerENoteIn;
	private DigitalInput _bimBreakerENoteOut;
	private DigitalInput _bimBreakerENOteIndexer;
	private DigitalInput _bimBreakerSH;
	private DigitalInput _LimitSwitchE;
	private DigitalInput _LimitSwitchC;
	private Timer _shootPrepareTimer = new Timer();



	public static StateMachine getInstance() {
		if (_instance == null) _instance = new StateMachine();

		return _instance;
	}

	private Map<RobotStates, StateEndCondition> _endConditionMap;

	private StateMachine() {
		_endConditionMap = new HashMap<>();

		for (RobotStates state : RobotStates.values()) _endConditionMap.put(state, new StateEndCondition(() -> false, RobotStates.IDLE));

		setEndConditionMap();
//		_bimBreakerENoteIn=new DigitalInput(Constants.ElevatorConstants.kbimBreakerNoteInID);
//		_bimBreakerENoteOut=new DigitalInput(Constants.ElevatorConstants.kbimBreakerNoteOutID);
		_bimBreakerENOteIndexer=new DigitalInput(Constants.IndexerConstants.kLimitSwitchID);
//		_LimitSwitchE=new DigitalInput(Constants.IndexerConstants.kLimitSwitchID);
		_LimitSwitchC=new DigitalInput(Constants.ClimberConstants.kLimitSwitchID);
		_bimBreakerSH=new DigitalInput(Constants.ShooterConstants.kbimBreakerNoteID);
	}

	/**
	 * Set in this function the end condition for each state with _endConditionMap
	 */
	private void setEndConditionMap(){
//		_endConditionMap.put(RobotStates.ELEVATOR_AMP_PREPARE,
//			new StateEndCondition(() -> Elevator.getInstance().atGoal(), RobotStates.ELEVATOR_AMP_READY));
//		_endConditionMap.put(RobotStates.ELEVATOR_TRAP_PREPARE,
//				new StateEndCondition(() -> Elevator.getInstance().atGoal(), RobotStates.ELEVATOR_TRAP_READY));
//		_endConditionMap.put(RobotStates.CLOSE,
//				new StateEndCondition(() -> Elevator.getInstance().atGoal(), RobotStates.IDLE));
//		_endConditionMap.put(RobotStates.NOTE_TO_ELEVATOR,
//				new StateEndCondition(() ->_LimitSwitchE.get() , RobotStates.ELEVATOR_TRAP_READY));
		_endConditionMap.put(RobotStates.INTAKE,
				new StateEndCondition(() -> RobotState.getbimBreakerIndexer(), RobotStates.NOTE_IN_INDEXER));

		_endConditionMap.put(RobotStates.SHOOT_AMP_PREPARE,
				new StateEndCondition(() ->_shootPrepareTimer.advanceIfElapsed(1) , RobotStates.SHOOT_AMP_RADY));
		_endConditionMap.put(RobotStates.SHOOT,
				new StateEndCondition(() -> !RobotState.getbimBreakerShoot(), RobotStates.CLOSE));


		_endConditionMap.put(RobotStates.SHOOT_PREPARE,
				new StateEndCondition(() ->_shootPrepareTimer.advanceIfElapsed(1), RobotStates.SHOOT_READY));
		_endConditionMap.put(RobotStates.SHOOT_READY,
				new StateEndCondition(() -> !RobotState.getbimBreakerShoot(), RobotStates.CLOSE));

		_endConditionMap.put(RobotStates.PREPARE_CLIMB,
				new StateEndCondition(() -> Climber.getInstance().atGoal(), RobotStates.CLIMB_READY));
		_endConditionMap.put(RobotStates.CLIMB,
				new StateEndCondition(() -> Climber.getInstance().atGoal(), RobotStates.CLIMBED));





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

			case CLOSE, RESET:
				if (wantedState == RobotStates.IDLE) RobotState.setRobotState(wantedState);
				break;

			case IDLE:
				if (wantedState == RobotStates.NOTE_IN_INDEXER
						|| wantedState == RobotStates.NOTE_SEARCH
						|| wantedState == RobotStates.RESET
						|| wantedState == RobotStates.PREPARE_CLIMB
						|| wantedState == RobotStates.SHOOT_PREPARE)) RobotState.setRobotState(wantedState);
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
			case NOTE_IN_INDEXER:
				if(wantedState == RobotStates.NOTE_TO_ELEVATOR ||wantedState == RobotStates.SHOOT_PREPARE)
					RobotState.setRobotState(wantedState);
				break;
			case NOTE_TO_ELEVATOR:
				if (wantedState == RobotStates.RESET
						|| wantedState == RobotStates.ELEVATOR_TRAP_PREPARE
						|| wantedState == RobotStates.ELEVATOR_AMP_PREPARE
						|| wantedState == RobotStates.PREPARE_CLIMB
						|| wantedState == RobotStates.SHOOT_PREPARE) RobotState.setRobotState(wantedState);
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
