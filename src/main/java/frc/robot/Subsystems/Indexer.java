package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.NinjasLib.NinjasSimulatedController;
import frc.robot.NinjasLib.NinjasSparkMaxController;
import frc.robot.NinjasLib.StateMachineMotoredSubsystem;
import frc.robot.Constants.IndexerConstants;
import frc.robot.RobotState;

public class Indexer extends StateMachineMotoredSubsystem {
    private static Indexer _instance;

    public static Indexer getInstance() {
        if (_instance == null) _instance = new Indexer();

        return _instance;
    }

    @Override
    protected void setController() {
        _controller = new NinjasSparkMaxController(IndexerConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {
        _simulatedController = new NinjasSimulatedController(IndexerConstants.kSimulatedControllerConstants);
    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
                () -> controller().setPercent(IndexerConstants.States.kRoll),
                RobotState.RobotStates.SHOOT);

        addFunctionToOnChangeMap(
                () -> controller().stop(),
                RobotState.RobotStates.CLOSE);

        addFunctionToOnChangeMap(
            () -> controller().setPercent(IndexerConstants.States.kRoll),
            RobotState.RobotStates.INTAKE);

        addFunctionToOnChangeMap(() -> resetSubsystem().schedule(), RobotState.RobotStates.RESET);
    }
}
