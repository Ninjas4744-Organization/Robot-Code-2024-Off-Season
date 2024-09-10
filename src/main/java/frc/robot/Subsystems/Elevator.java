package frc.robot.Subsystems;


import frc.robot.AbstractClasses.NinjasSparkMaxController;
import frc.robot.AbstractClasses.NinjasSubsystem;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends NinjasSubsystem {
    private static Elevator _instance1;
    public static Climber getInstance() {
		if (_instance == null) _instance = new Elevator();

		return _instance;
	}
    @Override
	protected void setController() {
		_controller = new NinjasSparkMaxController(ElevatorConstants.kControllerConstants);
	}
    



    
}
