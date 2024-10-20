package frc.robot.Subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasTalonFXController;
import frc.robot.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;

public class Shooter extends StateMachineMotoredSubsystem {
	private static Shooter _instance;

	public static Shooter getInstance() {
		if (_instance == null) _instance = new Shooter();

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
	public void resetSubsystem() {
		controller().stop();
	}

	@Override
	public boolean isResetted() {
		return controller().getOutput() == 0;
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(
			() -> controller().setVelocity(ShooterConstants.States.kSpeaker),
			RobotState.RobotStates.SHOOT_SPEAKER_PREPARE, RobotStates.SHOOT_SPEAKER_READY, RobotStates.OOGA_BOOGA, RobotStates.OOGA_BOOGA_READY);

		addFunctionToOnChangeMap(
			() -> controller().setVelocity(ShooterConstants.States.kAmp),
			RobotStates.SHOOT_AMP_PREPARE, RobotStates.SHOOT_AMP_READY);

		addFunctionToOnChangeMap(
			() -> controller().setVelocity(ShooterConstants.States.kDelivery),
			RobotStates.DELIVERY);

		addFunctionToOnChangeMap(() -> controller().setVelocity(ShooterConstants.States.kOuttake),
			RobotStates.OUTTAKE);

		addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET, RobotStates.CLOSE);
	}

	public boolean isReady() {
		return controller().getVelocity() >= controller().getGoal() - ShooterConstants.kMinimumShootTolerance;
	}
}
