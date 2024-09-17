package frc.robot.Subsystems;

import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants;
import frc.robot.RobotState;

public class Indexer extends NinjasSubsystem {
    private static Intake _instance;

    public static Intake getInstance() {
        if (_instance == null) _instance = new Intake();

        return _instance;
    }
    @Override
    protected void setController() {
        _controller = new NinjasSparkMaxController(Constants.RollersConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {

    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToPeriodicMap(
                () -> {
                    controller().setPercent(Constants.IndexerConstants.States.kElevator);
                },
                RobotState.RobotStates.INDEXER_TO_ELEVATOR);
        addFunctionToPeriodicMap(
                () -> {
                    controller().setPercent(Constants.IndexerConstants.States.kShoot);
                },
                RobotState.RobotStates.);
        addFunctionToOnChangeMap(
                () -> {
                    controller().stop();
                },
                RobotState.RobotStates.ELEVATOR_AMP_PREPARE,
                RobotState.RobotStates.ELEVATOR_TRAP_PREPARE,
                RobotState.RobotStates.RESET,
                RobotState.RobotStates.CLOSE);

    }
}
