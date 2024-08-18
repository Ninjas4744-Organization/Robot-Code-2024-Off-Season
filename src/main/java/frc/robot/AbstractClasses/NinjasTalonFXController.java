package frc.robot.AbstractClasses;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.DataClasses.MainControllerConstants;

public class NinjasTalonFXController extends NinjasController {
	private TalonFX _main;
	private TalonFX[] _followers;

	public NinjasTalonFXController(MainControllerConstants constants) {
		super(constants);

		_main = new TalonFX(constants.main.id);
		_main.getConfigurator()
				.apply(new TalonFXConfiguration()
						.withAudio(new AudioConfigs().withBeepOnBoot(true))
						.withMotorOutput(new MotorOutputConfigs()
								.withInverted(
										constants.main.inverted
												? InvertedValue.CounterClockwise_Positive
												: InvertedValue.Clockwise_Positive))
						.withMotionMagic(new MotionMagicConfigs()
								.withMotionMagicAcceleration(constants.PIDFConstants.kAcceleration)
								.withMotionMagicCruiseVelocity(constants.PIDFConstants.kCruiseVelocity))
						.withCurrentLimits(new CurrentLimitsConfigs()
								.withStatorCurrentLimit(constants.currentLimit)
								.withStatorCurrentLimitEnable(true)
								.withSupplyCurrentLimit(constants.currentLimit)
								.withSupplyCurrentLimitEnable(true))
						.withSlot0(new Slot0Configs()
								.withKP(constants.PIDFConstants.kP)
								.withKI(constants.PIDFConstants.kI)
								.withKD(constants.PIDFConstants.kD)));

		_followers = new TalonFX[constants.followers.length];
		for (int i = 0; i < _followers.length; i++) {
			_followers[i] = new TalonFX(constants.followers[i].id);
			_followers[i].getConfigurator().apply(new TalonFXConfiguration());
			_followers[i].setControl(new Follower(constants.main.id, constants.followers[i].inverted));
		}
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.set(percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

		_main.setControl(new MotionMagicVoltage(position / _constants.encoderConversionFactor));
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

		_main.setControl(new MotionMagicVelocityVoltage(velocity / (_constants.encoderConversionFactor / 60)));
	}

	@Override
	public double getPosition() {
		return _main.getPosition().getValue() * _constants.encoderConversionFactor;
	}

	@Override
	public double getVelocity() {
		return _main.getVelocity().getValue() * _constants.encoderConversionFactor / 60;
	}

	@Override
	public double getOutput() {
		return _main.get();
	}

	@Override
	public void setEncoder(double position) {
		_main.setPosition(position / _constants.encoderConversionFactor);
	}

	@Override
	public boolean atGoal() {
		if (_controlState == ControlState.PIDF_POSITION)
			return Math.abs(getGoal() - getPosition()) < _constants.positionGoalTolerance;
		else if (_controlState == ControlState.PIDF_VELOCITY)
			return Math.abs(getGoal() - getVelocity()) < _constants.velocityGoalTolerance;

		return false;
	}
}
