package frc.robot.NinjasLib.Controllers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.NinjasLib.DataClasses.MainControllerConstants;

public class NinjasSparkMaxController extends NinjasController {
	private final CANSparkMax _main;
	private final CANSparkMax[] _followers;

	private final TrapezoidProfile _profile;
	private final Timer _profileTimer = new Timer();
	private State _initialProfileState;
	private double pidSetpoint;

	public NinjasSparkMaxController(MainControllerConstants constants) {
		super(constants);

		_main = new CANSparkMax(constants.main.id, CANSparkMax.MotorType.kBrushless);

		_main.restoreFactoryDefaults();

		_main.setInverted(constants.main.inverted);
		_main.setSmartCurrentLimit((int) constants.currentLimit);

		_main.getPIDController().setP(constants.PIDFConstants.kP);
		_main.getPIDController().setI(constants.PIDFConstants.kI);
		_main.getPIDController().setD(constants.PIDFConstants.kD);
		_main.getPIDController().setIZone(constants.PIDFConstants.kIZone);

		_main.getEncoder().setPositionConversionFactor(constants.encoderConversionFactor);
		_main.getEncoder().setVelocityConversionFactor(constants.encoderConversionFactor / 60);

		_main.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, constants.isMaxSoftLimit);
		_main.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, constants.isMinSoftLimit);
		_main.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float) constants.maxSoftLimit);
		_main.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float) constants.minSoftLimit);

		_main.burnFlash();

		_followers = new CANSparkMax[constants.followers.length];
		for (int i = 0; i < _followers.length; i++) {
			_followers[i] = new CANSparkMax(constants.followers[i].id, CANSparkMax.MotorType.kBrushless);
			_followers[i].restoreFactoryDefaults();
			_followers[i].follow(_main, constants.followers[i].inverted);
			_followers[i].burnFlash();
		}

		_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
				constants.PIDFConstants.kCruiseVelocity, constants.PIDFConstants.kAcceleration));

		_initialProfileState = new State(getPosition(), getVelocity());
	}

	@Override
	public void setPercent(double percent) {
		super.setPercent(percent);

		_main.set(percent);
	}

	@Override
	public void setPosition(double position) {
		super.setPosition(position);

		if (_controlState == ControlState.PID_POSITION)
			_main.getPIDController().setReference(getGoal(), ControlType.kPosition);

		_profileTimer.restart();
		_initialProfileState = new State(getPosition(), getVelocity());
	}

	@Override
	public void setVelocity(double velocity) {
		super.setVelocity(velocity);

		if (_controlState == ControlState.PID_VELOCITY)
			_main.getPIDController().setReference(getGoal(), ControlType.kVelocity);

		_profileTimer.restart();
		_initialProfileState = new State(getPosition(), getVelocity());
	}

	@Override
	public double getPosition() {
		return _main.getEncoder().getPosition();
	}

	@Override
	public double getVelocity() {
		return _main.getEncoder().getVelocity();
	}

	@Override
	public double getOutput() {
		return _main.getBusVoltage() * _main.getAppliedOutput() / 12;
	}

	@Override
	public void setEncoder(double position) {
		_main.getEncoder().setPosition(position);
	}

	@Override
	public void periodic() {
		super.periodic();

		SmartDashboard.putNumber("Timer", _profileTimer.get() + (_constants.dynamicProfiling ? 0.1 : 0));
		SmartDashboard.putNumber("Initial Profile State Pos", _initialProfileState.position);
		SmartDashboard.putNumber("Initial Profile State Vel", _initialProfileState.velocity);
		SmartDashboard.putNumber("Goal State Pos", getGoal());
		SmartDashboard.putNumber(
				"Profile Wanted Pos",
				_profile.calculate(
								_profileTimer.get() + (_constants.dynamicProfiling ? 0.1 : 0),
								_initialProfileState,
								new State(getGoal(), 0))
						.position);

//		if (atGoal() || _profileTimer.get() > _profile.totalTime()) return;

		switch (_controlState) {
			case PIDF_POSITION:
//				pidSetpoint = _profile.calculate(
//					0.02,
//					new State(pidSetpoint, getVelocity()),
//					new State(getGoal(), 0))
//					.position;
//
//				_main.getPIDController().setReference(pidSetpoint, ControlType.kPosition);

				_main.getPIDController()
					.setReference(
						_profile.calculate(
							_profileTimer.get() + (_constants.dynamicProfiling ? 0.1 : 0),
							_initialProfileState,
							new State(getGoal(), 0))
							.position,
						ControlType.kPosition);
				break;

			case PIDF_VELOCITY:
				_main.getPIDController()
						.setReference(
								_profile.calculate(
												_profileTimer.get() + (_constants.dynamicProfiling ? 0.1 : 0),
												_initialProfileState,
												new State(getPosition(), getGoal()))
										.velocity,
								ControlType.kVelocity);
				break;

			case FF_POSITION:
				_main.set(_profile.calculate(
										_profileTimer.get() + (_constants.dynamicProfiling ? 0.1 : 0),
										_initialProfileState,
										new State(getGoal(), 0))
								.velocity
						* _constants.PIDFConstants.kV
						/ 12);
				break;

			case FF_VELOCITY:
				_main.set(_profile.calculate(
										_profileTimer.get() + (_constants.dynamicProfiling ? 0.1 : 0),
										_initialProfileState,
										new State(getPosition(), getGoal()))
								.velocity
						* _constants.PIDFConstants.kV
						/ 12);
				break;

			default:
				break;
		}
	}
}
