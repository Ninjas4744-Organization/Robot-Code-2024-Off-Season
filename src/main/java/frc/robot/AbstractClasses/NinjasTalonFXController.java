package frc.robot.AbstractClasses;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.DataClasses.MainControllerConstants;

public class NinjasTalonFXController extends NinjasController {
  private TalonFX _main;
  private TalonFX[] _followers;
  private double _positionConversionFactor;
  private double _velocityConversionFactor;

  public NinjasTalonFXController(MainControllerConstants constants) {
    super(constants);

    _main = new TalonFX(constants.main.id);
    _main
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(
                            constants.main.inverted
                                ? InvertedValue.CounterClockwise_Positive
                                : InvertedValue.Clockwise_Positive))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(constants.PIDFConstants.kAcceleration)
                        .withMotionMagicCruiseVelocity(constants.PIDFConstants.kCruiseVelocity))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(constants.currentLimit)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(constants.currentLimit)
                        .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                    new Slot0Configs()
                        .withKP(constants.PIDFConstants.kP)
                        .withKI(constants.PIDFConstants.kI)
                        .withKD(constants.PIDFConstants.kD)));

    _positionConversionFactor = constants.encoderConversionFactor;
    _velocityConversionFactor = constants.encoderConversionFactor / 60;

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
  public State getEncoder() {
    return new State(
        _main.getPosition().getValue() * _positionConversionFactor,
        _main.getVelocity().getValue() * _velocityConversionFactor);
  }

  @Override
  public void setEncoder(double position) {
    _main.setPosition(position / _positionConversionFactor);
  }

  @Override
  public double getOutput() {
    return _main.get();
  }

  @Override
  public void periodic() {
    super.periodic();

    switch (_controlState) {
      case PIDF_POSITION:
        _main.setControl(
            new PositionVoltage(
                _profile.calculate(_trapozoidTimer.get(), getEncoder(), getGoal()).position));
        break;

      case PIDF_VELOCITY:
        _main.setControl(
            new VelocityVoltage(
                _profile.calculate(_trapozoidTimer.get(), getEncoder(), getGoal()).velocity));
        break;

      case MOTION_MAGIC_POSITION:
        _main.setControl(new MotionMagicVoltage(getGoal().position));
        break;

      case MOTION_MAGIC_VELOCITY:
        _main.setControl(new MotionMagicVelocityVoltage(getGoal().velocity));
        break;

      default:
        break;
    }
  }

  @Override
  public boolean atGoal() {
    if (_controlState == ControlState.PIDF_POSITION)
      return Math.abs(getGoal().position - getEncoder().position) < _goalTolerance.position;
    else if (_controlState == ControlState.PIDF_VELOCITY)
      return Math.abs(getGoal().velocity - getEncoder().velocity) < _goalTolerance.velocity;

    return false;
  }
}
