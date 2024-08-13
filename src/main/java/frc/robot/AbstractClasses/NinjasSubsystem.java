package frc.robot.AbstractClasses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class NinjasSubsystem extends SubsystemBase {
    protected NinjasController _group;
    RobotStates _currentState;

    
    public NinjasSubsystem(NinjasController groupOfMotors){
        _group = groupOfMotors;
    }

    public Command runMotor(double percetage){
        return Commands.startEnd(
                ()->{
                    _group.setPercent(percetage);
                },
                _group::stop,
              this);
    }

    

    @Override
    public void periodic() {
        switch (_currentState) {
            case INTAKE:
              intake();
              break;
            case STOP:
              _group.stop();
              break;
            case MANUAL:
              manual();
              break;
            case RESET:
              reset();
              break;
            default:
              idle();
              break;
          }
        _group.periodic();
    }

    public abstract void intake();
    public abstract void idle();
    public abstract void manual();
    public abstract void reset();
    public abstract void POOPING();
    public abstract void CLIMBING();
}
