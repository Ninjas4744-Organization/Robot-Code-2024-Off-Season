package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.pathFollowingConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.VisionIO;
import frc.robot.subsystems.VisionIOPhoton;
// import frc.robot.subsystems.VisionIOPhoton;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Intake.Elevator;
import frc.robot.subsystems.Intake.Rollers;
import frc.robot.subsystems.Intake.Rotation;

public class RobotContainer {
  // Subsystems
  private Swerve _swerve;
  private Vision _vision;
  private Climber _climber;
  private Elevator _elevator;
  private Rotation _rotation;
  private Rollers _rollers;
  private VisionSubsystem _photonVision;

  // Misc
  private CommandBuilder _commandBuilder;
  private CommandPS5Controller _joystick;
  private CommandPS5Controller _joystick2;
  private boolean Tornado = false;

  public RobotContainer() {
    _joystick = new CommandPS5Controller(Constants.kJoystickPort);
    _joystick2 = new CommandPS5Controller(Constants.kJoystick2Port);

    // _vision = new Vision();
    VisionIOPhoton[] cameras = { new VisionIOPhoton("Front", new Transform3d(0.37, 0, 0, new Rotation3d())) };
    _photonVision = new VisionSubsystem(cameras);

    _swerve = new Swerve(_photonVision::estimationSupplier, _photonVision::hasTargets);

    _climber = new Climber();
    _elevator = new Elevator();
    _rotation = new Rotation();
    _rollers = new Rollers();

    _commandBuilder = new CommandBuilder(_swerve, _climber, _elevator, _rotation, _rollers);

    NamedCommands.registerCommand("Outake", _commandBuilder.runAutoOutake());
    NamedCommands.registerCommand("Reset", _commandBuilder.Reset());

    configureBindings();
  }

  private void configureBindings() {
    configureDriverBindings();
    configureOperatorBindings();
    configureManualBindings();
  }

  private void configureMiscBindings(){

  }

  private void configureDriverBindings(){
    _swerve.setDefaultCommand(
      new TeleopSwerve(
        _swerve,
        () -> { return -_joystick.getLeftY() * Constants.Swerve.kDriveCoefficient; },
        () -> { return -_joystick.getLeftX() * Constants.Swerve.kDriveCoefficient; },
        () -> { return -_joystick.getRightX() * Constants.Swerve.kDriveCoefficient * Constants.Swerve.kDriveRotationCoefficient; },
        () -> { return -_joystick.getRightY() * Constants.Swerve.kDriveCoefficient * Constants.Swerve.kDriveRotationCoefficient; },
        () -> { return false; },
        () -> { return Tornado; }
      )
    );

    // _joystick.R2().whileTrue(Commands.defer(() -> {
    //     return GoToTag();
    //   }, Set.of(_swerve, _vision))
    // );

    _joystick.R3().toggleOnTrue(Commands.startEnd(
      () -> {Tornado = true;},
      () -> {Tornado = false;}
    ));

    _joystick.L1().onTrue(Commands.runOnce(() -> { _swerve.zeroGyro(); _swerve.zeroModules(); }, _swerve));
  }

  private void configureOperatorBindings(){
    // _joystick2.cross().onTrue(Commands.select(getAcceptCommands(), () -> {return getAcceptId();}));

    // _joystick2.circle().onTrue(
    //   Commands.parallel(
    //     _elevator.Reset(),
    //     _rotation.Reset()
    //   )
    // );

    // _joystick2.square().onTrue(
    //   Commands.parallel(
    //     _elevator.runClose(),
    //     _rotation.runOpen(Constants.Rotation.States.kUpRotation)
    //   )
    // );

    _joystick.cross().onTrue(
      Commands.parallel(
        _elevator.Reset(),
        _rotation.Reset()
      )
    );

    _joystick.square().onTrue(
      Commands.parallel(
        _elevator.runClose(),
        _rotation.runOpen(Constants.Rotation.States.kUpRotation)
      )
    );

    _joystick.triangle().onTrue(_commandBuilder.runInOutTake(
        Constants.Elevator.States.kAmpOpenHeight, Constants.Rotation.States.kAmpOpenRotation,
        _joystick.getHID()::getTriangleButton));

    _joystick.circle().onTrue(_commandBuilder.runOutake(Constants.Elevator.States.kTrapOpenHeight-0.04,
        Constants.Rotation.States.kTrapOpenRotation + 13, _joystick.getHID()::getCircleButton));
  }

  private void configureManualBindings(){
    // _joystick2.povRight().whileTrue(
    //   Commands.startEnd(
    //     () -> {_rollers.setMotor(1);},
    //     () -> {_rollers.Stop();},
    //     _rollers
    //   )
    // );

    // _joystick2.povLeft().whileTrue(
    //   Commands.startEnd(
    //     () -> {_rollers.setMotor(-1);},
    //     () -> {_rollers.Stop();},
    //     _rollers
    //   )
    // );

    _joystick2.povUp().whileTrue(
      Commands.startEnd(
        () -> {_climber.setMotor(1);},
        () -> {_climber.setMotor(0);},
        _climber
      )
    );

    _joystick2.povDown().whileTrue(
      Commands.startEnd(
        () -> {_climber.setMotor(-1);},
        () -> {_climber.setMotor(0);},
        _climber
      )
    );

    // _joystick2.R2().whileTrue(
    //   Commands.startEnd(
    //     () -> {_elevator.setMotor(0.4);},
    //     () -> {_elevator.Stop();},
    //     _elevator
    //   )
    // );

    // _joystick2.L2().whileTrue(
    //   Commands.startEnd(
    //     () -> {_elevator.setMotor(-0.4);},
    //     () -> {_elevator.Stop();},
    //     _elevator
    //   )
    // );

    // _joystick2.R1().whileTrue(
    //   Commands.startEnd(
    //     () -> {_rotation.setMotor(0.1);},
    //     () -> {_rotation.Stop();},
    //     _rotation
    //   )
    // );

    // _joystick2.L1().whileTrue(
    //   Commands.startEnd(
    //     () -> {_rotation.setMotor(-0.07);},
    //     () -> {_rotation.Stop();},
    //     _rotation
    //   )
    // );
  }

  public void periodic(){
    SmartDashboard.putNumber("Photon X", _photonVision.getRobotPose().getX());
    SmartDashboard.putNumber("Photon Y", _photonVision.getRobotPose().getY());
    SmartDashboard.putNumber("Photon 0", _photonVision.getRobotPose().getRotation().getDegrees());
  }

  private HashMap<Integer, Command> getAcceptCommands() {
    HashMap<Integer, Command> _acceptCommands = new HashMap<Integer, Command>();

    // Source
    _acceptCommands.put(1, _commandBuilder.runIntake());
    _acceptCommands.put(2, _commandBuilder.runIntake());
    _acceptCommands.put(9, _commandBuilder.runIntake());
    _acceptCommands.put(10, _commandBuilder.runIntake());

    // Amp
    // for(int i = 5; i <= 6; i++)
    //   _acceptCommands.put(i, _commandBuilder.runOutake(
    //     Constants.Elevator.States.kAmpOpenHeight,
    //     Constants.Rotation.States.kAmpOpenRotation,
    //     _joystick2.getHID()::getCrossButton)
    //   );

    // // Stage
    // for(int i = 11; i <= 16; i++)
    //   _acceptCommands.put(i, _commandBuilder.runOutake(Constants.Elevator.States.kTrapOpenHeight,
    //       Constants.Rotation.States.kTrapOpenRotation, _joystick2.getHID()::getCrossButton));

    return _acceptCommands;
  }

  private int getAcceptId() {
    int tag = _vision.getTag().ID;
    Optional<Alliance> ally = DriverStation.getAlliance();
    List<Integer> blueIDs = Arrays.asList(6, 14, 15, 16, 1, 2);
    List<Integer> redIDs = Arrays.asList(5, 11, 12, 13, 9, 10);

    if (ally.get() == Alliance.Blue) {
      if (blueIDs.contains(tag))
        return tag;
    } else {
      if (redIDs.contains(tag))
        return tag;
    }

    return -1;
  }

  public void Reset(){
    _commandBuilder.Reset().schedule();

    // double Setpoint = Math.PI / 2;
    // if(_vision.isTag()){
    //   Commands.run(() -> {
    //     try (PIDController _aController = new PIDController(Constants.Swerve.smartAngleKP, Constants.Swerve.smartAngleKI, Constants.Swerve.smartAngleKD)) {
    //       _aController.enableContinuousInput(-1.5 * Math.PI, 0.5 * Math.PI);
    //       System.out.println("current: " + _swerve.getLastCalculatedPosition().getRotation().getRadians());
    //       System.out.println("setpoint: " + Setpoint);
    //       _swerve.drive(
    //         new Translation2d(0, 0),
    //         _aController.calculate(_swerve.getLastCalculatedPosition().getRotation().getRadians(), Setpoint),
    //         false,
    //         true
    //       );
    //     }
    //   }, _swerve, _vision)
    //   .until(() -> {return Math.abs(_swerve.getLastCalculatedPosition().getRotation().getRadians() - Setpoint) <= 0.01;})
    //   .andThen(() -> {_swerve.zeroGyro();}, _swerve)
    //   .schedule();
    // }
  }

  public Command autoCommand(String auto) {
    return _commandBuilder.autoCommand(auto);
  }

  public Command GoToTag(){
    Pose2d targetPos = _vision.getTagPose();
    // return _commandBuilder.runInOutTake(Constants.Elevator.States.kAmpOpenHeight, Constants.Rotation.States.kAmpOpenRotation,
    //     _joystick.getHID()::getTriangleButton)
    // .andThen(GoTo(targetPos, 0.25))
    // .andThen(GoTo(targetPos, 0.25));
    return GoTo(targetPos, 0.25);
  }

  public Command GoTo(Pose2d targetPos, double Offset){
    Pose2d currentPos = _swerve.getLastCalculatedPosition();

    Pose2d distFromTag = new Pose2d(
      Offset * Math.cos(targetPos.getRotation().getRadians()),
      Offset * Math.sin(targetPos.getRotation().getRadians()),
      new Rotation2d()
    );

    targetPos = new Pose2d(targetPos.getX() + distFromTag.getX(), targetPos.getY() + distFromTag.getY(), targetPos.getRotation());

    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(currentPos, targetPos);

    PathPlannerPath path = new PathPlannerPath(
      bezierPoints,
      Constants.AutoConstants.constraints,
      new GoalEndState(0, targetPos.getRotation().plus(Rotation2d.fromDegrees(180)))
    );
    
    return Commands.sequence(
      Commands.runOnce(() -> _swerve.resetOdometry(currentPos), _swerve),
      AutoBuilder.followPath(path)
    );
  }
}