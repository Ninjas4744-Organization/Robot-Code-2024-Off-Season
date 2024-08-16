package frc.robot.Swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SwerveTest {
    static final double DELTA = 1e-2;

    @BeforeEach
    void setup() {
        assert HAL.initialize(500, 0);
    }

    @Test
    public void driveStraight() {
        Swerve.getInstance().drive(new ChassisSpeeds(1.0, 0.0, 0.0));
        for (SwerveModuleState module : Swerve.getInstance().getModuleStates()) {
            assertEquals(1.0, module.speedMetersPerSecond, DELTA);
            assertEquals(-90.0, module.angle.getDegrees(), DELTA);
        }
    }

    @Test
    public void driveStrafe() {
        Swerve.getInstance().drive(new ChassisSpeeds(0.0, 1.0, 0.0));
        for (SwerveModuleState module : Swerve.getInstance().getModuleStates()) {
            assertEquals(-1.0, module.speedMetersPerSecond, DELTA);
            assertEquals(-90.0, module.angle.getDegrees(), DELTA);
        }
    }
}
