package frc.robot;

public class Ballistics {
  // Constants
  static final double g = 9.81; // gravitational acceleration (m/s^2)
  static final double epsilon = 1e-6; // precision for binary search

  /**
   * Calculates the shooting angle needed.
   *
   * @param h     The height of the shooter (m)
   * @param d     The horizontal distance to the target (m)
   * @param H     The height of the target (m)
   * @param v     The speed of the shooter (m/s)
   * @param theta The angle of entrance to the target (degrees)
   * @return The angle to shoot at (degrees)
   */
  public static double calculateShootingAngle(double h, double d, double H, double v, double theta) {
    // Convert theta to radians for calculations
    double targetAngle = Math.toRadians(theta);

    // Define bounds for the shooting angle (in radians)
    double low = 0;
    double high = Math.PI / 2;

    // Binary search for the correct angle
    while (high - low > epsilon) {
      double mid = (low + high) / 2;
      double yAtD = calculateYAtDistance(mid, h, d, v);

      // Check the derivative at the target to match the angle of entry
      double derivative = calculateDerivativeAtDistance(mid, h, d, v);

      if (Math.abs(derivative - Math.tan(targetAngle)) < epsilon) {
        return Math.toDegrees(mid);
      }

      if (yAtD < H) {
        low = mid;
      } else {
        high = mid;
      }
    }

    return Math.toDegrees((low + high) / 2);
  }

  // Calculates the y-position at a given distance d for angle alpha
  private static double calculateYAtDistance(double alpha, double h, double d, double v) {
    double term1 = d * Math.tan(alpha);
    double term2 = (g * Math.pow(d, 2)) / (2 * Math.pow(v * Math.cos(alpha), 2));
    return h + term1 - term2;
  }

  // Calculates the derivative of the y-position with respect to distance at distance d
  private static double calculateDerivativeAtDistance(double alpha, double h, double d, double v) {
    double term1 = Math.tan(alpha);
    double term2 = (g * d) / (Math.pow(v * Math.cos(alpha), 2));
    return term1 - term2;
  }
}
