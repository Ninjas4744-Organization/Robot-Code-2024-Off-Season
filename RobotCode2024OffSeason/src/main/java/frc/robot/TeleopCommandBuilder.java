package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake.Elevator;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Intake.Rotation;

public class TeleopCommandBuilder {
  private Swerve _swerve;
  private Climber _climber;
  private Elevator _elevator;
  private Rotation _rotation;
  private Rollers _rollers;

  public TeleopCommandBuilder(Swerve swerve, Climber climber, Elevator elevator, Rotation rotation, Rollers rollers){
    _swerve = swerve;
    _climber = climber;
    _elevator = elevator;
    _rotation = rotation;
    _rollers = rollers;
  }

  public Command runInOutTake(double elevatorHeight, double rotation, BooleanSupplier condition) {
    return Commands.either(
      runIntake(),
      runOutake(elevatorHeight, rotation, condition),
      () -> { return !_rollers.isNote(); }
    );
  }

  public Command runIntake() {
    return Commands.sequence(
      Commands.parallel(
        _elevator.runOpen(Constants.Elevator.States.kSourceOpenHeight),
        _rotation.runOpen(Constants.Rotation.States.kSourceOpenRotation)
      ),

      _rollers.runIntake().until(_rollers::isNote),

      Commands.parallel(
        _elevator.runClose(),
        _rotation.runOpen(Constants.Rotation.States.kUpRotation)
      )
    );
  }

  public Command runOutake(double height, double rotation, BooleanSupplier condition) {
    return Commands.sequence(
      Commands.parallel(
        _elevator.runOpen(height),
        _rotation.runOpen(rotation)
      ),
      Commands.waitUntil(condition),

      Commands.either(
        _rollers.runIntake(0.4).raceWith(Commands.waitSeconds(Constants.Rollers.kTimeToOutake)),
        _rollers.runIntake().raceWith(Commands.waitSeconds(Constants.Rollers.kTimeToOutake)),
        () -> {return height == Constants.Rotation.States.kTrapOpenRotation;}
      ),

      Commands.parallel(
        _elevator.runClose(),
        Commands.either(
          _rotation.runOpen(Constants.Rotation.States.kUpRotation),
          _rotation.runClose(),
          () -> {return height == Constants.Rotation.States.kTrapOpenRotation;}
        )
      )
    );
  }

  public Command runAutoOutake() {
    return Commands.sequence(
      Commands.parallel(
        _elevator.runOpen(Constants.Elevator.States.kAmpOpenHeight),
        _rotation.runOpen(Constants.Rotation.States.kAmpOpenRotation)
      ),

      Commands.waitUntil(
        () -> {
          return _elevator.isHeight(Constants.Elevator.States.kAmpOpenHeight)
              && _rotation.isRotation(Constants.Rotation.States.kAmpOpenRotation);
        }
      ),

      Commands.waitSeconds(0.1),

      _rollers.runIntake().raceWith(Commands.waitSeconds(Constants.Rollers.kTimeToOutake))

      // Commands.parallel(
      //   _elevator.runClose(),
      //   _rotation.runOpen(Constants.Rotation.States.kUpRotation)
      // )
    );
  }

  public Command autoCommand(String auto) {
    // PathPlannerPath _path = PathPlannerPath.fromPathFile("Score");
    // _swerve.resetOdometry(_path.getPreviewStartingHolonomicPose());

    return AutoBuilder.buildAuto(auto);
  }

  public Command Reset() {
    _swerve.zeroGyro();
    _swerve.resetOdometry(new Pose2d());

    return Commands.parallel(
      _elevator.Reset(),
      _rotation.Reset(),
      _climber.Reset()
  ).until(() -> {return _elevator.isHeight(0) && _rotation.isRotation(0) && _climber.isLimitSwitch();});
  }
}