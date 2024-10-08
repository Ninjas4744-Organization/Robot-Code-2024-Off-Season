package frc.robot.Subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.RobotState.RobotStates;

public class Shooter extends StateMachineMotoredSubsystem {
	private static Shooter _instance;

	public static Shooter getInstance() {
		if (_instance == null) _instance = new Shooter();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ShooterConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(ShooterConstants.kSimulatedControllerConstants);
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
				() -> controller().setVelocity(ShooterConstants.States.kSpeaker), RobotStates.SHOOT_SPEAKER_PREPARE);

		addFunctionToOnChangeMap(
				() -> controller().setVelocity(ShooterConstants.States.kAmp), RobotStates.SHOOT_AMP_PREPARE);

		addFunctionToOnChangeMap(() -> controller().stop(), RobotStates.CLOSE);
	}
}
