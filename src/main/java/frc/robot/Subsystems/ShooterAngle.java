package frc.robot.Subsystems;

import com.ninjas4744.NinjasLib.Controllers.NinjasSimulatedController;
import com.ninjas4744.NinjasLib.Controllers.NinjasSparkMaxController;
import com.ninjas4744.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.RobotStates;

public class ShooterAngle extends StateMachineMotoredSubsystem<RobotStates> {
	private static ShooterAngle _instance;
	private DigitalInput _limit;

	public ShooterAngle() {
		super();
		_limit = new DigitalInput(ShooterAngleConstants.kLimitSwitchId);
	}

	public static ShooterAngle getInstance() {
		if (_instance == null) _instance = new ShooterAngle();

		return _instance;
	}

	@Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ShooterAngleConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {
		_simulatedController = new NinjasSimulatedController(ShooterAngleConstants.kSimulatedControllerConstants);
	}

	@Override
	public void resetSubsystem() {
		runMotor(-0.35).until(_limit::get).schedule();
	}

	@Override
	public boolean isResetted() {
		return _limit.get();
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToPeriodicMap(
			() -> controller().setPosition(ShooterAngleConstants.States.Amp.get()), RobotStates.SHOOT_AMP_PREPARE, RobotStates.SHOOT_AMP_READY);

		addFunctionToPeriodicMap(
				() -> controller()
						.setPosition(
								ShooterAngleConstants.calculateLaunchAngle(ShooterAngleConstants.getSpeakerHolePose())
										.getDegrees()),
				RobotStates.SHOOT_SPEAKER_PREPARE,
				RobotStates.SHOOT_SPEAKER_READY);

		addFunctionToPeriodicMap(() -> controller().setPosition(ShooterAngleConstants.States.Up.get()), RobotStates.OOGA_BOOGA, RobotStates.OOGA_BOOGA_READY);

		addFunctionToPeriodicMap(() -> controller().setPosition(ShooterAngleConstants.States.Delivery.get()), RobotStates.DELIVERY);

		addFunctionToOnChangeMap(this::resetSubsystem, RobotStates.RESET);
		addFunctionToPeriodicMap(() -> controller().setPosition(ShooterAngleConstants.States.Close.get()), RobotStates.CLOSE);
	}

	@Override
	public void periodic() {
		super.periodic();

		SmartDashboard.putBoolean("Shooter Angle Limit", _limit.get());
		if (_limit.get()) {
			controller().resetEncoder();
			if (controller().getOutput() < 0) controller().stop();
		}
	}
}
