package org.team199.lib;

public class SwerveMath {
    public static SwerveContainer[] calculateSwerve(double forward, double strafe, double rotation, double length, double width) {
        // Calculate helper variables
        double r = Math.sqrt(length * length + width * width);
        double a = strafe - rotation * (length / r);
        double b = strafe + rotation * (length / r);
        double c = forward - rotation * (width / r);
        double d = forward + rotation * (width / r);

        // Calculate wheel speeds for each side and normalize
        double[] wheelSpeeds = {Math.sqrt(b * b + d * d), Math.sqrt(b * b + c * c),
                                Math.sqrt(a * a + d * d), Math.sqrt(a * a + c * c)};
        wheelSpeeds = normalize(wheelSpeeds);

        // Calculate angles for each side
        double[] angles = {Math.atan2(d, b), Math.atan2(c, b), Math.atan2(d, a), Math.atan2(c, a)};

        // Create and return SwerveContainers
        SwerveContainer[] swerveContainers = new SwerveContainer[4];
        for (int i = 0; i < swerveContainers.length; i++) {
            swerveContainers[i] = new SwerveContainer(wheelSpeeds[i], angles[i]);
        }
        return swerveContainers;
    }

    // Heading must be in radians.
    public static SwerveContainer[] calculateSwerve(double forward, double strafe, double rotation, double heading, double length, double width) {
        double newForward = Math.cos(heading) * forward + Math.sin(heading) * strafe;
        double newStrafe = -Math.sin(heading) * forward + Math.cos(heading) * strafe;
        return calculateSwerve(newForward, newStrafe, rotation, length, width);
    }

    public static double[] normalize(double[] x) {
        // Determine the maximum maginitude of x.
        double maximum = -Double.MAX_VALUE;
        for (int i = 0; i < x.length; i++) {
            // Must take absolute value in order to preserve original signs.
            if (Math.abs(x[i]) > maximum) { maximum = Math.abs(x[i]); }
        }

        // If the maximum magnitude is less than 1, normalizing is unnecessary.
        if ((maximum <= 1.0) && (0.0 < maximum)) { return x; }

        double[] normalized_x = new double[x.length];
        // The only way the maximum is zero is if all of the elements are zero or near-zero.
        if (maximum == 0.0) {
            for (int i = 0; i < x.length; i++) {
                normalized_x[i] = 0.0;
            }
        // Divide by the maximum magnitude to normalize.
        } else {
            for (int i = 0; i < x.length; i++) {
                normalized_x[i] = x[i] / maximum;
            }
        }
        return normalized_x;
    }
}