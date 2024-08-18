package frc.robot.Swerve;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;

public class SwerveTest {
	static final double DELTA = 1e-2;

	@BeforeEach
	void setup() {
		assert HAL.initialize(500, 0);
	}

	//  @Test
	//  public void driveStraight() {
	//    Swerve.getInstance().drive(new ChassisSpeeds(1.0, 0.0, 0.0));
	//    for (SwerveModuleState module : Swerve.getInstance().getModuleStates()) {
	//      assertEquals(1.0, module.speedMetersPerSecond, DELTA);
	//      assertEquals(-90.0, module.angle.getDegrees(), DELTA);
	//    }
	//  }
	//
	//  @Test
	//  public void driveStrafe() {
	//    Swerve.getInstance().drive(new ChassisSpeeds(0.0, 1.0, 0.0));
	//    for (SwerveModuleState module : Swerve.getInstance().getModuleStates()) {
	//      assertEquals(-1.0, module.speedMetersPerSecond, DELTA);
	//      assertEquals(-90.0, module.angle.getDegrees(), DELTA);
	//    }
	//  }
}
