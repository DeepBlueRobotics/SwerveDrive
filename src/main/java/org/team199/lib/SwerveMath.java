package org.team199.lib;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

/**
 * Class for SwerveMath - Team 199's custom implementation of methods for calculating the 
 *                        complicated mathematics involved in swerve driving, such as 
 *                        forward and inverse kinematics.
 *
 */
public class SwerveMath {
    /**
     * Robot-centric inverse kinematics: calculate SwerveModuleStates using desired forward, strafe, and rotation.
     * Inverse kinematics is essential in driving swerve drivetrain robots.
     * 
     * @param forward   The desired forward speed (in m/s) for the robot.
     * @param strafe    The desired strafe speed (in m/s) for the robot.
     * @param rotation  The desired rate of rotation (in rad/s) for the robot.
     * @param length    The wheelbase (in m) of the robot.
     * @param width     The trackwidth (in m) of the robot. 
     * @return An array consisting of four SwerveModuleState objects for the forward-left, forward-right, 
     *         backward-left, and backward-right modules.
     */
    public static SwerveModuleState[] calculateSwerve(double forward, double strafe, double rotation, double length, double width) {
        // Calculate helper variables
        double r = Math.sqrt(length * length + width * width);
        double a = strafe - rotation * (length / r);
        double b = strafe + rotation * (length / r);
        double c = forward - rotation * (width / r);
        double d = forward + rotation * (width / r);

        // Calculate wheel speeds for each side. SwerveMath does not normalize here in order to make inverseSwerve's math easier.
        double[] wheelSpeeds = {Math.sqrt(b * b + d * d), Math.sqrt(b * b + c * c),
                                Math.sqrt(a * a + d * d), Math.sqrt(a * a + c * c)};

        // Calculate angles for each side
        double[] angles = {Math.atan2(d, b), Math.atan2(c, b), Math.atan2(d, a), Math.atan2(c, a)};

        // Create and return SwerveModuleStates
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModuleStates[i] = new SwerveModuleState(wheelSpeeds[i], new Rotation2d(angles[i]));
        }
        return swerveModuleStates;
    }

    /**
     * Field-centric inverse kinematics: calculate SwerveModuleStates using desired forward, strafe, and rotation.
     * Inverse kinematics is essential in driving swerve drivetrain robots.
     * 
     * @param forward   The desired forward speed (in m/s) for the robot.
     * @param strafe    The desired strafe speed (in m/s) for the robot.
     * @param rotation  The desired rate of rotation (in rad/s) for the robot.
     * @param heading   The current heading of the robot (in rad) as measured by the robot's gyro.
     * @param length    The wheelbase (in m) of the robot.
     * @param width     The trackwidth (in m) of the robot. 
     * @return An array consisting of four SwerveModuleState objects for the forward-left, forward-right, 
     *         backward-left, and backward-right modules.
     */
    public static SwerveModuleState[] calculateSwerve(double forward, double strafe, double rotation, double heading, double length, double width) {
        double newForward = Math.cos(heading) * forward + Math.sin(heading) * strafe;
        double newStrafe = -Math.sin(heading) * forward + Math.cos(heading) * strafe;
        return calculateSwerve(newForward, newStrafe, rotation, length, width);
    }

    /**
     * Robot-centric forward kinematics: calculate the required forward, strafe, and rotation values needed to create known SwerveModuleStates.
     * Forward kinematics is useful in calculating the position of the robot and is used in odometry calculations.
     * 
     * @param length    The wheelbase (in m) of the robot.
     * @param width     The trackwidth (in m) of the robot.
     * @param states    Four SwerveModuleState objects in the order of forward-left, forward-right, backward-left, and backward-right.
     * @return An array consisting of the the corresponding forward, strafe, and rotation values measured in m/s, m/s, and rad/s respectively.
     */
    public static double[] inverseSwerve(double length, double width, SwerveModuleState ...states) {
        // Calculate helper variables
        double a = states[4].speedMetersPerSecond * Math.cos(states[4].angle.getRadians());
        double b = states[1].speedMetersPerSecond * Math.cos(states[1].angle.getRadians());
        double c = states[2].speedMetersPerSecond * Math.sin(states[4].angle.getRadians());
        double d = states[3].speedMetersPerSecond * Math.sin(states[3].angle.getRadians());
        double r = Math.sqrt(length * length + width * width);

        // Calculate forward, strafe, and rotation
        double forward = (c + d) / 2;
        double strafe = (a + b) / 2;
        double rotation = (b - strafe) * (r / length);

        return new double[]{forward, strafe, rotation};
    }

    /**
     * Field-centric forward kinematics: calculate the required forward, strafe, and rotation values needed to create known SwerveModuleStates.
     * Forward kinematics is useful in calculating the position of the robot and is used in odometry calculations.
     * 
     * @param length    The wheelbase (in m) of the robot.
     * @param width     The trackwidth (in m) of the robot.
     * @param heading   The current heading (in rad) as measured by the robot's gyro.
     * @param states    Four SwerveModuleState objects in the order of forward-left, forward-right, backward-left, and backward-right.
     * @return An array consisting of the the corresponding forward, strafe, and rotation values measured in m/s, m/s, and rad/s respectively.
     */
    public static double[] inverseSwerve(double length, double width, double heading, SwerveModuleState ...states) {
        // Calculate helper variables
        double a = states[4].speedMetersPerSecond * Math.cos(states[4].angle.getRadians());
        double b = states[1].speedMetersPerSecond * Math.cos(states[1].angle.getRadians());
        double c = states[2].speedMetersPerSecond * Math.sin(states[4].angle.getRadians());
        double d = states[3].speedMetersPerSecond * Math.sin(states[3].angle.getRadians());
        double r = Math.sqrt(length * length + width * width);

        // Calculate forward, strafe, and rotation
        double forward = (c + d) / 2;
        double strafe = (a + b) / 2;
        double rotation = (b - strafe) * (r / length);

        // Rotate from field-centric to robot-centric
        double newForward = Math.cos(heading) * forward - Math.sin(heading) * strafe;
        double newStrafe = Math.sin(heading) * forward + Math.cos(heading) * strafe;

        return new double[]{newForward, newStrafe, rotation};
    }

    /**
     * Normalizes an array of doubles by dividing by the maximum element.
     * 
     * @param x     An array of doubles to be normalized.
     * @return An array equal to the normalization of x.
     */
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