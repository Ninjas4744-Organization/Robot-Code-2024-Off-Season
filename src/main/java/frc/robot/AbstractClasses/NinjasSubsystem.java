package frc.robot.AbstractClasses;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public abstract class NinjasSubsystem extends SubsystemBase {
  protected NinjasController _controller;

  // /**
  //  * Resets the subsystem- moves the subsystem down until limit hit and then stops.
  //  * @return the command that does that
  //  */
  // public Command resetSubsystem() {
  //   return runMotor(-0.5);
  // }

  public Command runMotor(double percent) {
    return Commands.startEnd(
      () -> _controller.setPercent(percent), 
      () -> _controller.stop(), 
      this);
  }

  @Override
  public void periodic() {
    switch(RobotState.getRobotState()) {
      case IDLE:
        idle();
        break;
        
      case PREPARE_INTAKE:
        prepareIntake();
        break;

      case INTAKE:
        intake();
        break;

      case PREPARE_AMP_OUTAKE:
        prepareAmpOutake();
        break;
        
      case PREPARE_TRAP_OUTAKE:
        prepareTrapOutake();
        break;

      case OUTAKE:
        outake();
        break;

      case CLOSE:
        close();
        break;

      case RESET:
        reset();
        break;

      case PREPARE_SHOOT:
        prepareShoot();
        break;

      case SHOOT:
        shoot();
        break;

      case NOTE_SEARCH:
        noteSearch();
        break;

      case HOLDING_NOTE:
        holdingNote();
        break;

      case PREPARE_CLIMB:
        prepareClimb();
        break;

      case CLIMB:
        climb();
        break;
    }

    _controller.periodic();
  }

  protected abstract void idle();
  protected abstract void prepareIntake();
  protected abstract void intake();
  protected abstract void prepareAmpOutake();
  protected abstract void prepareTrapOutake();
  protected abstract void outake();
  protected abstract void outakeClose();
  protected abstract void close();
  protected abstract void reset();
  protected abstract void prepareShoot();
  protected abstract void shoot();
  protected abstract void noteSearch();
  protected abstract void holdingNote();
  protected abstract void prepareClimb();
  protected abstract void climb();
}
