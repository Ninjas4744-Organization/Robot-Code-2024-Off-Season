package frc.robot.AbstractClasses;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.DataClasses.ControllerConstants;

public class NinjasSparkMaxController extends NinjasController
{
    private CANSparkMax _controller;
    private TrapezoidProfile _profile;
    private SparkPIDController _pidController;

    public NinjasSparkMaxController(ControllerConstants constants)
    {
        super(constants);
        
        _controller = new CANSparkMax(constants.id, CANSparkMax.MotorType.kBrushless);
        _controller.restoreFactoryDefaults();
        _controller.setInverted(constants.invertOutput);
        _pidController = _controller.getPIDController();
        _pidController.setP(constants.pidConstants.kP);
        _pidController.setI(constants.pidConstants.kI);
        _pidController.setD(constants.pidConstants.kD);
        _pidController.setIZone(constants.pidConstants.iZone);
        _profile = new TrapezoidProfile(constants.constraints);
        _controller.burnFlash();
    }

    @Override
    public double get() {
        switch (_controlState) {
            case PERCENT_OUTPUT:
                return _controller.get();
        
            case MOTION_MAGIC:
                return -1;
        }
    }

    @Override
    public void set(double value) {

    }

    @Override
    public void stop() {
        
    }

    @Override
    public double getEncoder() {
        
    }

    @Override
    public void setEncoder(double position) {
        
    }

    @Override
    public double getSetpoint() {
        
    }

    @Override
    protected void updateShuffleboard() {
        
    }
    
}
