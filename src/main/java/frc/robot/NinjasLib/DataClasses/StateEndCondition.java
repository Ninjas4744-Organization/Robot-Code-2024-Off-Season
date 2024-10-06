package frc.robot.NinjasLib.DataClasses;

import frc.robot.RobotState.RobotStates;
import java.util.function.BooleanSupplier;

public class StateEndCondition {
	public BooleanSupplier condition;
	public RobotStates nextState;

	public StateEndCondition(BooleanSupplier condition, RobotStates nextState) {
		this.condition = condition;
		this.nextState = nextState;
	}
}
