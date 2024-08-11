package frc.robot.AbstractClasses;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DataClasses.MainControllerConstants;

public class NinjasSparkMaxController extends NinjasController
{
    private CANSparkMax _main;
    private CANSparkMax[] _followers;
    protected State _goalTolerance;
    
    protected TrapezoidProfile _profile;
    protected State _profileGoal;
    protected Timer _trapozoidTimer = new Timer();
    private RelativeEncoder _encoder;
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

    _main = new CANSparkMax(constants.main.id, CANSparkMax.MotorType.kBrushless);

    _main.restoreFactoryDefaults();

    _main.setInverted(constants.main.inverted);
    _main.setSmartCurrentLimit((int) constants.currentLimit);

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
        super._controlState = ControlState.PERCENT_OUTPUT;
        _main.set(percent);
    }
  

  

    @Override
    public boolean atGoal() {
        if(_controlState == ControlState.PIDF_POSITION)
            return Math.abs(_demand - getEncoder().position) < _goalTolerance.position;
        else if(_controlState == ControlState.PIDF_VELOCITY)
            return Math.abs(_demand - getEncoder().velocity) < _goalTolerance.velocity;

        return false;
    }

    @Override
    public void setPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
    }

    @Override
    public void setVelocity(double velocity) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocity'");
    }

    @Override
    public State getEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEncoder'");
    }

    @Override
    public void setEncoder(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setEncoder'");
    }
}
