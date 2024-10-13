package frc.robot.NinjasLib.Subsystems;import edu.wpi.first.wpilibj2.command.Command;import edu.wpi.first.wpilibj2.command.Commands;import frc.robot.NinjasLib.Controllers.NinjasController;import frc.robot.NinjasLib.Controllers.NinjasSimulatedController;import frc.robot.RobotState;public abstract class StateMachineMotoredSubsystem extends StateMachineSubsystem {	protected NinjasController _controller;	protected NinjasSimulatedController _simulatedController;	public StateMachineMotoredSubsystem(boolean disabled) {		super(disabled);		if (!disabled) {			if (RobotState.isSimulated()) setSimulationController();			else setController();		}	}	protected NinjasController controller() {		if (RobotState.isSimulated()) return _simulatedController;		else return _controller;	}	/**	 * Set the real controller of the subsystem.	 *	 * <p>Implement controller in the _controller variable,	 */	protected abstract void setController();	/**	 * Set the simulated controller of the subsystem.	 *	 * <p>Implement controller in the _simulatedController for the simulated one.	 *	 * <p>The simulated controller is optional, only set it if code will be simulated.	 */	protected abstract void setSimulationController();	/**	 * Resets the subsystem: moves the subsystem down until limit hit and then stops.	 *	 * @return the command that does that	 */	public abstract void resetSubsystem();	public abstract boolean isResetted();	/**	 * @return Whether the subsystem is homed/reseted/closed	 */	public boolean isHomed() {		if (disabled)			return false;		return controller().isHomed();	}	/**	 * @return Whether the subsystem is at its PIDF goal	 */	public boolean atGoal() {		if (disabled)			return false;		return controller().atGoal();	}	/**	 * Runs the motor at the given percent.	 *	 * @param percent - how much to power the motor between -1 and 1	 * @return a command that runs that on start and stops to motor on end	 */	public Command runMotor(double percent) {		if (disabled)			return Commands.none();		return Commands.startEnd(				() -> controller().setPercent(percent), () -> controller().stop(), this);	}	@Override	public void periodic() {		super.periodic();		if (disabled)			return;		controller().periodic();	}}