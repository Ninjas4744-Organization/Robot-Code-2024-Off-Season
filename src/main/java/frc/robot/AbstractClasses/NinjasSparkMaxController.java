package frc.robot.AbstractClasses;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;

import frc.robot.DataClasses.MainControllerConstants;

public class NinjasSparkMaxController extends NinjasController
{
    private CANSparkMax _main;
    private CANSparkMax[] _followers;

    public NinjasSparkMaxController(MainControllerConstants constants)
    {
        super(constants);
        
        _main = new CANSparkMax(constants.main.id, CANSparkMax.MotorType.kBrushless);
        
        _main.restoreFactoryDefaults();
        
        _main.setInverted(constants.main.inverted);
        _main.setSmartCurrentLimit((int)constants.currentLimit);
        
        _main.getPIDController().setP(constants.PIDFConstants.kP);
        _main.getPIDController().setI(constants.PIDFConstants.kI);
        _main.getPIDController().setD(constants.PIDFConstants.kD);
        _main.getPIDController().setIZone(constants.PIDFConstants.kIZone);
        _main.getPIDController().setFF(constants.PIDFConstants.kF);
        
        _main.burnFlash();

        _encoder = _main.getEncoder();
        _encoder.setPositionConversionFactor(constants.encoderConversionFactor);
        _encoder.setVelocityConversionFactor(constants.encoderConversionFactor / 60);

        _followers = new CANSparkMax[constants.followers.length];
        for (int i = 0; i < _followers.length; i++) {
            _followers[i] = new CANSparkMax(constants.followers[i].id, CANSparkMax.MotorType.kBrushless);
            _followers[i].restoreFactoryDefaults();
            _followers[i].follow(_main, constants.followers[i].inverted);
            _followers[i].burnFlash();
        }
    }

    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        _main.set(percent);
    }

    @Override
    public double getOutput(){
        return _main.get();
    }
    
    @Override
    public void periodic() {
        super.periodic();
        
        switch (_controlState) {
            case PIDF_POSITION:
                _main.getPIDController().setReference(_profile.calculate(_trapozoidTimer.get(), getEncoder(), getGoal()).position, ControlType.kPosition);
                break;

            case PIDF_VELOCITY:
                _main.getPIDController().setReference(_profile.calculate(_trapozoidTimer.get(), getEncoder(), getGoal()).velocity, ControlType.kVelocity);
                break;

            default:
                break;
        }
    }

    @Override
    public boolean atGoal() {
        if(_controlState == ControlState.PIDF_POSITION)
            return Math.abs(getGoal().position - getEncoder().position) < _goalTolerance.position;
        else if(_controlState == ControlState.PIDF_VELOCITY)
            return Math.abs(getGoal().velocity - getEncoder().velocity) < _goalTolerance.velocity;

        return false;
    }
}
