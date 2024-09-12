package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants;
import frc.robot.RobotState;

public class ElevatorIntake extends NinjasSubsystem {
    private static ElevatorIntake _instance;

    private Timer _outakeTimer = new Timer();

    public static ElevatorIntake getInstance() {
        if (_instance == null) _instance = new ElevatorIntake();

        return _instance;
    }

    @Override
    protected void setController() {
        _controller = new NinjasSparkMaxController(Constants.RollersIntakeConstants.kControllerConstants);
    }

    @Override
    protected void setSimulationController() {

    }

    @Override
    protected void setFunctionMaps() {
        addFunctionToOnChangeMap(
                () -> {
                    controller().setPercent(Constants.RollersIntakeConstants.States.kIntake);
                },
                RobotState.RobotStates.ELEVATOR_INTAKE);
        addFunctionToOnChangeMap(
                () -> {
                    controller().stop();
                },
                RobotState.RobotStates.ELEVATOR_AMP_PREPARE, RobotState.RobotStates.ELEVATOR_TRAP_PREPARE);
    }


}
