package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.NinjasSimulatedController;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.RobotState;
import frc.robot.StateMachine;

public class ShooterFeeder extends NinjasSubsystem {
	private static ShooterFeeder _instance;

	public static ShooterFeeder getInstance() {
		if (_instance == null) _instance = new ShooterFeeder();

		return _instance;
	}

	Timer shootTimer = new Timer();

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ShooterFeederConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(ShooterFeederConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
				() -> {
					controller().setPercent(1);
					shootTimer.restart();
				},
				RobotState.RobotStates.SHOOT);

		addFunctionToPeriodicMap(
				() -> {
					if (shootTimer.get() > 0.5) {
						StateMachine.getInstance().changeRobotState(RobotState.RobotStates.IDLE);
						controller().stop();
					}
				},
				RobotState.RobotStates.SHOOT);
	}
}