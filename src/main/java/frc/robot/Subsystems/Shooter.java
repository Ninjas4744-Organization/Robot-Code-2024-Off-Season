package frc.robot.Subsystems;

import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasTalonFXController;
import frc.robot.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
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
		addFunctionToPeriodicMap(
				() -> controller()
						.setVelocity(
								/*ShooterConstants.calculateLaunchSpeed(ShooterAngleConstants.calculateLaunchAngle(ShooterAngleConstants.getSpeakerHolePose()).getDegrees(), ShooterAngleConstants.getSpeakerHolePose())*/ 90),
				RobotStates.SHOOT_SPEAKER_PREPARE);

		addFunctionToPeriodicMap(
				() -> controller()
						.setVelocity(ShooterConstants.calculateLaunchSpeed(
								ShooterAngleConstants.calculateLaunchAngle(ShooterAngleConstants.getAmpHolePose())
										.getDegrees(),
								ShooterAngleConstants.getAmpHolePose())),
				RobotStates.SHOOT_AMP_PREPARE);

		addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
	}
}
