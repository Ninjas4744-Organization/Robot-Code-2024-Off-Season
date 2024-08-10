package frc.robot.AbstractClasses;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.DataClasses.ControllerConstants;

public class NinjasTalonFXController extends NinjasController {
  private TalonFX _controller;
  private TalonFX[] _followers;

  public NinjasTalonFXController(
      ControllerConstants constants, ControllerConstants... followersConstants) {
    super(constants, followersConstants);

    _controller = new TalonFX(constants.id);
    _controller
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withInverted(
                            constants.invertOutput
                                ? InvertedValue.CounterClockwise_Positive
                                : InvertedValue.Clockwise_Positive))
                .withMotionMagic(
                    new MotionMagicConfigs()
                        .withMotionMagicAcceleration(constants.constraints.maxAcceleration)
                        .withMotionMagicCruiseVelocity(constants.constraints.maxVelocity))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(constants.currentLimit)
                        .withStatorCurrentLimitEnable(true)
                        .withSupplyCurrentLimit(constants.currentLimit)
                        .withSupplyCurrentLimitEnable(true))
                .withSlot0(
                    new Slot0Configs()
                        .withKP(constants.pidConstants.kP)
                        .withKI(constants.pidConstants.kI)
                        .withKD(constants.pidConstants.kD)));

    _followers = new TalonFX[followersConstants.length];
    for (int i = 0; i < _followers.length; i++) {
      _followers[i] = new TalonFX(followersConstants[i].id);
      _followers[i]
          .getConfigurator()
          .apply(
              new TalonFXConfiguration()
                  .withMotionMagic(
                      new MotionMagicConfigs()
                          .withMotionMagicAcceleration(
                              followersConstants[i].constraints.maxAcceleration)
                          .withMotionMagicCruiseVelocity(
                              followersConstants[i].constraints.maxVelocity))
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(followersConstants[i].currentLimit)
                          .withStatorCurrentLimitEnable(true)
                          .withSupplyCurrentLimit(followersConstants[i].currentLimit)
                          .withSupplyCurrentLimitEnable(true))
                  .withSlot0(
                      new Slot0Configs()
                          .withKP(followersConstants[i].pidConstants.kP)
                          .withKI(followersConstants[i].pidConstants.kI)
                          .withKD(followersConstants[i].pidConstants.kD)));
      _followers[i].setControl(
          new Follower(
              constants.id,
              constants.invertOutput
                  ? !followersConstants[i].invertOutput
                  : followersConstants[i].invertOutput));
    }
  }

  @Override
  public void setPercent(double percent) {
    super.setPercent(percent);
    _controller.set(percent);
  }

  @Override
  public State getEncoder() {
    return new State(_controller.getPosition().getValue(), _controller.getVelocity().getValue());
  }

  @Override
  public double getOutput() {
    return _controller.get();
  }

  @Override
  public void setEncoder(double position) {
    _controller.setPosition(position);
  }

  @Override
  public void periodic() {
    super.periodic();

    switch (_controlState) {
      case PIDF_POSITION:
        _controller.set(_pidfController.calculate(getEncoder().position, getSetpoint().position));
        break;

      case PIDF_VELOCITY:
        _controller.set(_pidfController.calculate(getEncoder().velocity, getSetpoint().velocity));
        break;

      default:
        break;
    }
  }
}
