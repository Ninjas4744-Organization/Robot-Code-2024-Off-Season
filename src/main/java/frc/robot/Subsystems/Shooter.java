package frc.robot.Subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasTalonFXController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.RobotState.RobotStates;

public class Shooter extends StateMachineMotoredSubsystem {
	private static Shooter _instance;

	public Shooter(boolean disabled) {
		super(disabled);
	}

	public static void disable() {
		if (_instance == null)
			_instance = new Shooter(true);
	}

	public static Shooter getInstance() {
		if (_instance == null) _instance = new Shooter(false);

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasTalonFXController(ShooterConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(ShooterConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
			() -> {
				System.out.println("SHOOTING!");
				controller().setVelocity(ShooterConstants.States.kSpeaker);
			}, RobotStates.SHOOT_SPEAKER_PREPARE);

		addFunctionToOnChangeMap(
				() -> controller().setVelocity(ShooterConstants.States.kAmp), RobotStates.SHOOT_AMP_PREPARE);

		addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.CLOSE);
	}
}
