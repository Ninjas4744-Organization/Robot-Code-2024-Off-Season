package frc.robot;

public class Ballistics {
  // Constants
  static final double g = 9.81; // gravitational acceleration (m/s^2)

  public static double findLaunchAngle(double H, double h, double d, double a) {
    double tanTerm = (2 * (H - h) / d) - Math.tan(a - 1);

      return Math.toDegrees(Math.atan(tanTerm));
  }

  public static double findLaunchSpeed(double h, double H, double d, double a) {
    double term1 = g * Math.pow(d, 2);
    double term2 = 2 * Math.pow(Math.cos(a), 2) * (d * Math.tan(a) - (H - h));

    return Math.sqrt(term1 / term2);
  }
}
