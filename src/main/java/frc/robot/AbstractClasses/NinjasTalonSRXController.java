package frc.robot.AbstractClasses;

import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.DataClasses.MainControllerConstants;

public class NinjasTalonSRXController extends NinjasController
{
    private TalonSRX _main;
    private TalonSRX[] _followers;
    private double _positionConversionFactor;
    private double _velocityConversionFactor;

    public NinjasTalonSRXController(MainControllerConstants constants)
    {
        super(constants);
        
        _main = new TalonSRX(constants.main.id);
        
        _main.configFactoryDefault();
        
        _main.setInverted(constants.main.inverted);
        _main.configPeakCurrentLimit((int)constants.currentLimit);
        
        _main.config_kP(0, constants.PIDFConstants.kP);
        _main.config_kI(0, constants.PIDFConstants.kI);
        _main.config_kD(0, constants.PIDFConstants.kD);
        _main.config_kF(0, constants.PIDFConstants.kF);

        _positionConversionFactor = constants.encoderConversionFactor;
        _velocityConversionFactor = constants.encoderConversionFactor / 60;

        _followers = new TalonSRX[constants.followers.length];
        for (int i = 0; i < _followers.length; i++) {
            _followers[i] = new TalonSRX(constants.followers[i].id);
            _followers[i].configFactoryDefault();
            _followers[i].follow(_main);
            _followers[i].setInverted(constants.followers[i].inverted ^ constants.main.inverted);
        }
    }

    @Override
    public void setPercent(double percent) {
        super.setPercent(percent);

        _main.set(TalonSRXControlMode.PercentOutput, percent);
    }

    @Override
    public State getEncoder(){
        return new State(
            _main.getSelectedSensorPosition() * _positionConversionFactor,
            _main.getSelectedSensorVelocity() * _velocityConversionFactor
        );
    }

    @Override
    public void setEncoder(double position){
        _main.setSelectedSensorPosition(position / _positionConversionFactor);
    }

    @Override
    public double getOutput(){
        return _main.getMotorOutputPercent();
    }
    
    @Override
    public void periodic() {
        super.periodic();
        
        switch (_controlState) {
            case PIDF_POSITION:
                _main.set(TalonSRXControlMode.Position, _profile.calculate(_trapozoidTimer.get(), getEncoder(), getGoal()).position);
                break;

            case PIDF_VELOCITY:
                _main.set(TalonSRXControlMode.Velocity, _profile.calculate(_trapozoidTimer.get(), getEncoder(), getGoal()).velocity);
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
