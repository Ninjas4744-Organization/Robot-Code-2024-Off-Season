package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Ballistics {
  // Constants
  static final double g = 9.81; // gravitational acceleration (m/s^2)

  public static double findLaunchAngle(double H, double h, double d, double a) {
    a = Units.degreesToRadians(a - 1);

    double tanTerm = (2 * (H - h) / d) - Math.tan(a);

    return Math.toDegrees(Math.atan(tanTerm));
  }

  public static double findLaunchSpeed(double h, double H, double d, double a) {
    a = Units.degreesToRadians(a);

    double term1 = g * Math.pow(d, 2);
    double term2 = 2 * Math.pow(Math.cos(a), 2) * (d * Math.tan(a) - (H - h));

    return Math.sqrt(term1 / term2);
  }
}
