package frc.robot.AbstractClasses;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.DataClasses.MasterConstants;

public class NinjasSparkMaxController extends NinjasController
{
    protected final CANSparkMax _master;
  protected final CANSparkMax[] _slaves;

  protected final RelativeEncoder _relEncoder;
  // protected final AbsoluteEncoder _absEncoder;
  protected final SparkPIDController _controller;

  private final TrapezoidProfile _profile;
  private final Timer trapozoidTimer = new Timer();

    double demand;

    public NinjasSparkMaxController(MasterConstants constants)
    {
        super(constants);
        
        _profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(constants.kCruiseVelocity,constants.kAcceleration));
        _master = new CANSparkMax(constants.master.id, CANSparkMax.MotorType.kBrushless);
        _master.restoreFactoryDefaults();
        _master.setInverted(constants.master.inverted);
        _master.setSmartCurrentLimit((int)constants.currentLimit);
        _master.burnFlash();

        _relEncoder = _master.getEncoder();
        _controller = _master.getPIDController();

        _relEncoder.setPositionConversionFactor(constants.kGearRatio);
        _relEncoder.setVelocityConversionFactor(constants.kGearRatio/60);

        _controller.setP(constants.Kp);
        _controller.setI(constants.Ki);
        _controller.setD(constants.Kd);
        _controller.setIZone(constants.KIzone);

        _master.burnFlash();

        _slaves = new CANSparkMax[constants.slaves.length];
        for (int i = 0; i < _slaves.length; i++) {
            _slaves[i] = new CANSparkMax(constants.slaves[i].id, CANSparkMax.MotorType.kBrushless);
            _slaves[i].restoreFactoryDefaults();
            _slaves[i].setInverted(constants.slaves[i].inverted);
            _slaves[i].follow(_master);
            _slaves[i].burnFlash();
        }
    }

    @Override
    public void setPercent(double percent) {
        _master.set(percent);
    }

    @Override
    public State getEncoder() {
        return new State(_relEncoder.getPosition(), _relEncoder.getVelocity());
    }

    @Override
    public double getOutput(){
        return 0;
        // return _controller.get(); ???
    }

    @Override
    public void setEncoder(double position) {
        _relEncoder.setPosition(position);
    }
    
    @Override
    public void periodic() {
        super.periodic();
        
        switch (_controlState) {
            case PIDF_POSITION:
                
                _controller.setReference(_profile.calculate(trapozoidTimer.get(), getEncoder(), new State(demand, 0)).position, ControlType.kPosition);
                break;

            default:
                break;
        }
    }

    @Override
    public State getSetpoint() {
        return new State(demand,demand);
    }

    @Override
    public boolean atSetpoint() {
        return false;
    }

    @Override
    public void setPosition(double position) {
        demand = position;
        _controller.setReference(position, ControlType.kPosition);
    }

    @Override
    public void setVelocity(double velocity) {
        demand = velocity;
        _controller.setReference(velocity, ControlType.kVelocity);
    }
}
