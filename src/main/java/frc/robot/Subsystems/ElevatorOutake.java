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

        addFunctionToOnChangeMap(
                () -> {
                    controller().setPercent(Constants.RollersIntakeConstants.States.kOutake);
                },
                RobotState.RobotStates.ELEVATOR_AMP_READY, RobotState.RobotStates.ELEVATOR_TRAP_READY);
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
