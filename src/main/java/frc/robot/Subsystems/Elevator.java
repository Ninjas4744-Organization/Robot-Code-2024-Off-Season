package frc.robot.Subsystems;


import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.RobotState;

public class Elevator extends NinjasSubsystem {
    private static Elevator _instance;
    public static Elevator getInstance() {
		if (_instance == null) _instance = new Elevator();

		return _instance;
	}
    @Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ElevatorConstants.kControllerConstants);
	}

	@Override
	protected void setSimulationController() {}

	@Override
	protected void setFunctionMaps() {
		addFunctionToOnChangeMap(() -> {
			controller().setPosition(ElevatorConstants.States.kAmp);
		}, RobotState.RobotStates.ELEVATOR_AMP_PREPARE);

		addFunctionToOnChangeMap(() -> {
			controller().setPosition(ElevatorConstants.States.kTrap);
		}, RobotState.RobotStates.ELEVATOR_TRAP_PREPARE);

		addFunctionToOnChangeMap(() -> {
			controller().setPosition(ElevatorConstants.States.kClose);
		}, RobotState.RobotStates.CLOSE);
	}



}
