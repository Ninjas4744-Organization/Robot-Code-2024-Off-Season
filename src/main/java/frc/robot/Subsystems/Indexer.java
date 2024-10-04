package frc.robot.Subsystems;

import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.StateMachineMotoredSubsystem;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Indexer extends StateMachineMotoredSubsystem {
    private static Intake _instance;

    public static Intake getInstance() {
        if (_instance == null) _instance = new Intake();

        return _instance;
    }
    @Override
    protected void setController() {
        _controller = new NinjasSparkMaxController(Constants.IndexerConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {

    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
                () -> controller().setPercent(Constants.IndexerConstants.States.kRoll),
                RobotState.RobotStates.SHOOT);

        addFunctionToOnChangeMap(
                () -> controller().stop(),
                RobotState.RobotStates.CLOSE);

        addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotState.RobotStates.RESET);

    }
}
