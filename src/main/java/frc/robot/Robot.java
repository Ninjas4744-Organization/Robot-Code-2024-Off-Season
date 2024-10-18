package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	private Command _autoCommand;
	private RobotContainer _robotContainer;
	private LoggedDashboardChooser<Command> autoChooser;

	@Override
	public void robotInit() {
		Logger.recordMetadata("ProjectName", "Robot-Code-2024-Off-Season"); // Set a metadata value

		if (isReal()) {
			Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // Enables power distribution logging
		} else {
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
			Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
		}

// Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
		Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

		Pathfinding.setPathfinder(new LocalADStar());
		_robotContainer = new RobotContainer();

		autoChooser = new LoggedDashboardChooser<>("Auto", AutoBuilder.buildAutoChooser());
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		_robotContainer.periodic();
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		if(!RobotState.isSimulated())
			if(DriverStation.isFMSAttached())
				Shuffleboard.startRecording();

		_autoCommand = AutoCommandBuilder.autoCommand("Center 1");
		if (_autoCommand != null) _autoCommand.schedule();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {}

	@Override
	public void teleopInit() {
		if (_autoCommand != null) _autoCommand.cancel();

		if(!RobotState.isSimulated())
			if(DriverStation.isFMSAttached())
				Shuffleboard.startRecording();

		_robotContainer.resetSubsystems();
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {
		Shuffleboard.stopRecording();
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
		RobotState.setRobotState(RobotState.RobotStates.TESTING);
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}
}
