package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.StateMachine;

public class ElevatorOutake extends NinjasSubsystem {
    private static ElevatorOutake _instance;

    private Timer _outakeTimer = new Timer();

    public static ElevatorOutake getInstance() {
        if (_instance == null) _instance = new ElevatorOutake();

        return _instance;
    }

    @Override
    protected void setController() {
        _controller = new NinjasSparkMaxController(Constants.RollersIntakeConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {

    }
    protected void setFunctionMaps() {

        addFunctionToPeriodicMap(
                () -> {
                    controller().setPercent(Constants.RollersConstants.States.kOutake);

                    if (_outakeTimer.get() > 1) {
                        StateMachine.getInstance().changeRobotState(RobotState.RobotStates.CLOSE);
                    }
                },
                RobotState.RobotStates.ELEVATOR_OUTAKE);

        addFunctionToOnChangeMap(() -> _outakeTimer.restart(), RobotState.RobotStates.ELEVATOR_OUTAKE);
        addFunctionToOnChangeMap(() -> controller().stop(), RobotState.RobotStates.CLOSE, RobotState.RobotStates.RESET);
    }
}
