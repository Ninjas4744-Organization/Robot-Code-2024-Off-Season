package frc.robot.Subsystems;

import frc.robot.NinjasLib.NinjasSimulatedController;
import frc.robot.NinjasLib.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShooterAngleConstants;
import frc.robot.RobotState.RobotStates;

public class Climber extends StateMachineMotoredSubsystem {
    private static Climber _instance;

    public static Climber getInstance() {
        if (_instance == null) _instance = new Climber();

        return _instance;
    }

    @Override
    protected void setController() {
        _controller = new NinjasSparkMaxController(ShooterAngleConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(ShooterAngleConstants.kSimulatedControllerConstants);
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
            () -> controller().setPosition(ClimberConstants.States.kUp), RobotStates.CLIMB_PREPARE);

        addFunctionToOnChangeMap(
            () -> controller().setPosition(ClimberConstants.States.kClose), RobotStates.CLOSE, RobotStates.CLIMB);

        addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotStates.RESET);
    }
}
