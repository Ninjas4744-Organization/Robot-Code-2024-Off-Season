package frc.robot.AbstractClasses;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DataClasses.ControllerConstants;

public class NinjasSparkMaxController extends NinjasController
{
    private CANSparkMax _controller;

    public NinjasSparkMaxController(ControllerConstants constants)
    {
        super(constants);
        
        _controller = new CANSparkMax(constants.id, CANSparkMax.MotorType.kBrushless);
        _controller.restoreFactoryDefaults();
        _controller.setInverted(constants.invertOutput);
        _controller.burnFlash();
    }

    @Override
    public double get() {
        switch (_controlState) {
            case PERCENT_OUTPUT:
                return _controller.get();

            case POSITION_PIDF:
                return _controller.getEncoder().getPosition();

            case VELOCITY_PIDF:
                return _controller.getEncoder().getVelocity();

            default:
                return -1;
        }
    }

    @Override
    public void set(double value) {
        switch (_controlState) {
            case PERCENT_OUTPUT:
                _controller.set(value);
                break;
            
            case MOTION_MAGIC:
                break;

            case POSITION_PIDF:
                _pidfController.setGoal(new TrapezoidProfile.State(value, 0));
                break;

            case VELOCITY_PIDF:
                _pidfController.setGoal(new TrapezoidProfile.State(0, value));
                break;
        }
    }

    @Override
    public void stop() {
        _controller.stopMotor();
    }

    @Override
    public TrapezoidProfile.State getEncoder() {
        return new TrapezoidProfile.State(_controller.getEncoder().getPosition(), _controller.getEncoder().getVelocity());
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
        
        if(_controlState == ControlState.POSITION_PIDF)
            _controller.set(_pidfController.calculate(getEncoder().position, getSetpointState()));
        else if(_controlState == ControlState.VELOCITY_PIDF)
            _controller.set(_pidfController.calculate(getEncoder().velocity, getSetpointState()));
    }
}
