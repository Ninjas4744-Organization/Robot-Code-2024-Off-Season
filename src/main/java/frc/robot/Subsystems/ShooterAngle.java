package frc.robot.Subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ballistics;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;
import frc.robot.NinjasLib.Controllers.NinjasSparkMaxController;
import frc.robot.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.robot.RobotState;
import frc.robot.RobotState.RobotStates;

public class ShooterAngle extends StateMachineMotoredSubsystem {
	private static ShooterAngle _instance;
	private DigitalInput _limit;

	public ShooterAngle() {
		super();
		_limit = new DigitalInput(ShooterAngleConstants.kLimitSwitchId);
	}

	public static ShooterAngle getInstance() {
		if (_instance == null)
			_instance = new ShooterAngle();

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
		return isHomed();
	}

	@Override
	protected void setFunctionMaps() {
		addFunctionToPeriodicMap(
				() -> controller().setPosition(ShooterAngleConstants.calculateLaunchAngle(ShooterAngleConstants.getAmpHolePose()).getDegrees()),
				RobotStates.SHOOT_AMP_PREPARE);

		addFunctionToPeriodicMap(
			() -> controller().setPosition(ShooterAngleConstants.calculateLaunchAngle(ShooterAngleConstants.getSpeakerHolePose()).getDegrees()),
			RobotStates.SHOOT_SPEAKER_PREPARE);

//		addFunctionToOnChangeMap(
//			() -> controller().setPosition(40),
//			RobotStates.SHOOT_SPEAKER_PREPARE);
	}

	@Override
	public void periodic() {
		super.periodic();

		SmartDashboard.putBoolean("Shooter Angle Limit", _limit.get());
		if (_limit.get()) {
			controller().resetEncoder();
			if (controller().getOutput() < 0)
				controller().stop();
		}
	}
}
