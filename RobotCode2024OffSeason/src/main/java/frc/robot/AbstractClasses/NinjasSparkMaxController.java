package frc.robot.AbstractClasses;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.DataClasses.ControllerConstants;

public class NinjasSparkMaxController extends NinjasController
{
    private CANSparkMax _controller;
    private CANSparkMax[] _followers;

    public NinjasSparkMaxController(ControllerConstants constants, ControllerConstants... followersConstants)
    {
        super(constants, followersConstants);
        
        _controller = new CANSparkMax(constants.id, CANSparkMax.MotorType.kBrushless);
        _controller.restoreFactoryDefaults();
        _controller.setInverted(constants.invertOutput);
        _controller.setSmartCurrentLimit((int)constants.currentLimit);
        _controller.burnFlash();

        _followers = new CANSparkMax[followersConstants.length];
        for (int i = 0; i < _followers.length; i++) {
            _followers[i] = new CANSparkMax(followersConstants[i].id, CANSparkMax.MotorType.kBrushless);
            _followers[i].restoreFactoryDefaults();
            _followers[i].setInverted(followersConstants[i].invertOutput);
            _followers[i].setSmartCurrentLimit((int)followersConstants[i].currentLimit);
            _followers[i].follow(_controller);
            _followers[i].burnFlash();
        }
    }

    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);
        _controller.set(percent);
    }

    @Override
    public State getEncoder() {
        return new State(_controller.getEncoder().getPosition(), _controller.getEncoder().getVelocity());
    }

    @Override
    public double getOutput(){
        return _controller.get();
    }

    @Override
    public void setEncoder(double position) {
        _controller.getEncoder().setPosition(position);
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
