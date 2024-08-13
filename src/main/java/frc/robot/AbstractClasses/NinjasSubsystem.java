package frc.robot.AbstractClasses;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public abstract class NinjasSubsystem extends SubsystemBase {
  protected NinjasController _controller;

  @Override
  public void periodic() {
    // switch(RobotState.getRobotState()) {
    //   case RobotStates.
    // }

    _controller.periodic();
  }

  // /**
  //  * Moves the subsystem down 
  //  * @return
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
}
