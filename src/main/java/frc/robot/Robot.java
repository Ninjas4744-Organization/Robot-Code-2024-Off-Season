package frc.robot;

import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	private Command _autoCommand;
	private RobotContainer _robotContainer;
	public static Robot instance;

	@Override
	public void robotInit() {
		Pathfinding.setPathfinder(new LocalADStar());
		instance = this;
		_robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		_robotContainer.periodic();
//		SwerveIO.getInstance().afterPeriodic();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
//		if (!RobotState.isSimulated()) if (DriverStation.isFMSAttached()) Shuffleboard.startRecording();

		_autoCommand = AutoCommandBuilder.autoCommand("Example Auto");
		if (_autoCommand != null) _autoCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (_autoCommand != null) _autoCommand.cancel();

//		if (!RobotState.isSimulated()) if (DriverStation.isFMSAttached()) Shuffleboard.startRecording();

		_robotContainer.resetSubsystems();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {
//		Shuffleboard.stopRecording();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		RobotState.getInstance().setRobotState(RobotStates.TESTING);
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
