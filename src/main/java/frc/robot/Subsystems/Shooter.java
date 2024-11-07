package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasTalonFXController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotStates;

public class Shooter extends StateMachineMotoredSubsystem<RobotStates> {
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
			() -> controller().setVelocity(ShooterConstants.States.kSpeaker.get()),
			RobotStates.SHOOT_SPEAKER_PREPARE, RobotStates.SHOOT_SPEAKER_READY, RobotStates.OOGA_BOOGA, RobotStates.OOGA_BOOGA_READY);

		addFunctionToOnChangeMap(
			() -> controller().setVelocity(ShooterConstants.States.kAmp.get()),
			RobotStates.SHOOT_AMP_PREPARE, RobotStates.SHOOT_AMP_READY);

		addFunctionToOnChangeMap(
			() -> controller().setVelocity(ShooterConstants.States.kDelivery.get()),
			RobotStates.DELIVERY);

		addFunctionToOnChangeMap(() -> controller().setVelocity(ShooterConstants.States.kOuttake.get()),
			RobotStates.OUTTAKE);

		addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET, RobotStates.CLOSE);
	}

	public boolean isReady() {
		return controller().getVelocity() >= controller().getGoal() - ShooterConstants.kMinimumShootTolerance;
	}
}
