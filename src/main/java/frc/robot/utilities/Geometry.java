package frc.robot.utilities;

public class Geometry {
    /**
     * @param counts    Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */

    public static double rotationsToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio));
    }

    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */

    public static double degreesToRotation(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio));
        return ticks;
    }

    public double angleWrapper(double angle, boolean negative) {
        angle = angle % 360; // reduce the angle
        angle = (angle + 360) % 360; // force it to be the postive remainder, so that 0 <= angle <= 360

        return angle;
    }
}
 