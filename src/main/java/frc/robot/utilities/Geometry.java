package frc.robot.utilities;

public class Geometry {
    public double fromDegreestoRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    public double fromRadianstoDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    public double fromRotationstoDegrees(double rotations) {
        return rotations * 360;
    }

    public double fromDegreestoRotations(double degrees) {
        return degrees / 360;
    }

    public double fromRadianstoRotations(double radians) {
        return radians / (2 * Math.PI);
    }

    public double fromRotationstoRadians(double rotations) {
        return rotations * 2 * Math.PI;
    }

    public double angleWrapper(double angle) {
        while (angle < 0) {
            angle += 360;
        }
        while (angle >= 360) {
            angle -= 360;
        }
        return angle;
       
    }

    public double radiansWrapper(double radians) {
        while (radians < 0) {
            radians += 2 * Math.PI;
        }
        while (radians >= 2 * Math.PI) {
            radians -= 2 * Math.PI;
        }
        return radians;
    }

    public double  rotationWrapper(double rotations) {
        while (rotations < 0) {
            rotations += 1;
        }
        while (rotations >= 1) {
            rotations -= 1;
        }
        return rotations;
    }
}
