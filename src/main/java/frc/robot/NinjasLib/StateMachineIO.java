package frc.robot.NinjasLib;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.NinjasLib.DataClasses.StateEndCondition;
import frc.robot.NinjasLib.Subsystems.StateMachineSubsystem;
import frc.robot.RobotState;

import java.util.HashMap;
import java.util.Map;

public abstract class StateMachineIO<StateEnum> extends StateMachineSubsystem {
    private static StateMachineIO _instance;
    private Map<StateEnum, StateEndCondition<StateEnum>> _endConditionMap;

    public static StateMachineIO getInstance() {
        if(_instance == null)
            throw new RuntimeException("StateMachineIO not initialized. Initialize StateMachineIO by setInstance() first.");
        return _instance;
    }

    public static void setInstance(StateMachineIO instance) {
        _instance = instance;
    }

    private StateMachineIO() {
        super();
        _endConditionMap = new HashMap<>();
        setEndConditionMap();
    }

    public void setTriggerForSimulationTesting(Trigger trigger) {
        if (RobotStateIO.getInstance().isSimulated())
            trigger.onTrue(Commands.runOnce(
                () -> changeRobotState(_endConditionMap.get(RobotStateIO.getInstance().getRobotState()).nextState)));
    }

    /**
     * Sets the state of the robot to the given state only if possible. For example if the current
     * state is AMP_OUTAKE_READY it cannot change to PREPARE_AMP_OUTAKE
     *
     * @param wantedState - the state to change the robot state to
     */
    public abstract void changeRobotState(StateEnum wantedState);

    /**
     * Set in this function the end condition for each state with _endConditionMap
     */
    protected abstract void setEndConditionMap();

    @Override
    public void periodic() {
        super.periodic();

        if (_endConditionMap.get(RobotStateIO.getInstance().getRobotState()).condition.getAsBoolean() &&
            !RobotState.isSimulated() &&
            _endConditionMap.get(RobotStateIO.getInstance().getRobotState()) != null)
            changeRobotState(_endConditionMap.get(RobotStateIO.getInstance().getRobotState()).nextState);
    }
}