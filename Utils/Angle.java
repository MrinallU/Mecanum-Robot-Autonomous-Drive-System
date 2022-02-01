package org.firstinspires.ftc.teamcode.Utils;

/**
 * Provides utilities for angle calculations.
 */
public class Angle {
    // Remaps the given angle onto the range [0, 360).
    public static double normalizePositive(double degrees) {
        double normalized_angle = degrees % 360;
        if (normalized_angle < 0) {
            normalized_angle += 360;
        }
        return normalized_angle;
    }

    // Remaps the given angle into the range (-180, 180].
    public static double normalize(double degrees) {
        double normalized_angle = Angle.normalizePositive(degrees);
        if (normalized_angle > 180) {
            normalized_angle -= 360;
        }
        return normalized_angle;
    }

    // Remaps the given angle into the range (-90, 90].
    public static double normalize_half(double degrees) {
        return Angle.normalize(2 * degrees) / 2;
    }

    /**
     * Compute the difference between the given two angles, in degrees.
     * Output is in the range (-180, 180].
     */
    public static double angleDifference(double start, double end) {
        double raw_delta = end - start;
        return Angle.normalize(raw_delta);
    }


    // Convert degrees to radians.
    public static double degrees_to_radians(double degrees) {
        return Math.PI * (degrees/180);
    }

    // Convert radians to degrees.
    public static double radians_to_degrees(double radians) {
        return 180 * (radians/ Math.PI);
    }
}
