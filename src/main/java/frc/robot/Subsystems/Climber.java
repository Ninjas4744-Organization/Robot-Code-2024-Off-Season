package frc.robot.Subsystems;

import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.RobotState.RobotStates;
import frc.robot.StateMachine;

public class Climber extends NinjasSubsystem {
	private static Climber _instance;

	public static Climber getInstance() {
		if (_instance == null) _instance = new Climber();

		return _instance;
	}

	private Climber() {
		super();

		_controller = new NinjasSparkMaxController(ClimberConstants.kControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(() -> _controller.setPosition(ClimberConstants.States.kUp), RobotStates.PREPARE_CLIMB);

		addFunctionToPeriodicMap(
				() -> {
					if (_controller.atGoal()) StateMachine.getInstance().changeRobotState(RobotStates.CLIMB_READY);
				},
				RobotStates.PREPARE_CLIMB);

		addFunctionToOnChangeMap(
				() -> _controller.setPosition(ClimberConstants.States.kClose), RobotStates.CLOSE, RobotStates.CLIMB);
		addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotStates.RESET);
	}
}
