package frc.robot.DataClasses;

public class SimulatedControllerConstants {
	/** Regular controller constants */
	public MainControllerConstants mainControllerConstants = new MainControllerConstants();

	/** Torque of the motor in its subsystem(jKgMetersSquared) */
	public double motorTorque = 1;

	/**
	 * Gear ratio between the motor output and the output after the gearbox, bigger values means
	 * bigger reduction(1 / x)
	 */
	public double gearRatio = 1;
}
