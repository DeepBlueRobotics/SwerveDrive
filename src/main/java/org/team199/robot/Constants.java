/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot;

public final class Constants {
    // One inch = 0.0254 meters
    public static double inchToMeter = 0.0254;

    public static final class Ports {
        // CAN ids for each of the drive motor controllers.
        public static final int kDriveFrontLeft = 0;
        public static final int kDriveFrontRight = 15;
        public static final int kDriveBackLeft = 1;
        public static final int kDriveBackRight = 14;
        
        // CAN ids for each of the turn motor controllers.
        public static final int kTurnFrontLeft = 4;
        public static final int kTurnFrontRight = 11;
        public static final int kTurnBackLeft = 5;
        public static final int kTurnBackRight = 10;
    }

    public static final class DriveConstants {
        // Original units are in inches and must be converted to meters.
        public static final double wheelBase = 18.15 * inchToMeter;
        public static final double trackWidth = 17.75 * inchToMeter;

        // The maximum drive speed (in ft/s for a CIM motor) is listed on the AndyMark page for the Swerve and Steer module.
        // maxForward and maxStrafe are in m/s, maxRCW is in radians, and maxSpeed is in m/s.
        // Speed is the same for maxForward, maxStrafe, and maxSpeed since they are all equal to the max speed for the drive motor 
        public static final double maxForward = 11.5 * 12 * inchToMeter;
        public static final double maxStrafe = 11.5 * 12 * inchToMeter;
        public static final double maxSpeed = 11.5 * 12 * inchToMeter;
        // maxRCW is the angular velocity of the robot.
        // Calculated by looking at one of the motors and treating it as a point mass moving around in a circle.
        // Tangential speed of this point mass is maxSpeed and the radius of the circle is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
        // Angular velocity = Tangential speed / radius
        public static final double maxRCW = maxSpeed / Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));

        // kP, kI, and kD constants for drive motor controllers in the order of front-left, front-right, back-left, back-right.
        public static final double[] driveKP = {20.0, 20.0, 20.0, 20.0};
        public static final double[] driveKI = {0.1, 0.1, 0.1, 0.1};
        public static final double[] driveKD = {0.0, 0.0, 0.0, 0.0};

        // kP, kI, and kD constants for turn motor controllers in the order of front-left, front-right, back-left, back-right.
        public static final double[] turnKP = {20.0, 20.0, 20.0, 20.0};
        public static final double[] turnKI = {0.1, 0.1, 0.1, 0.1};
        public static final double[] turnKD = {0.0, 0.0, 0.0, 0.0};

        // Drivetrain Characterization constants in the order of front-left, front-right, back-left, back-right.
        public static final double[] kVels = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kAccels = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kVolts = {0.0, 0.0, 0.0, 0.0};

        public static final double kDriveModifier = 0.6;

        // Whether or not steering should be reversed
        public static final boolean reversedFL = false;
        public static final boolean reversedFR = false;
        public static final boolean reversedBL = false;
        public static final boolean reversedBR = false;

        // All of the gearboxes are PG71. PG71 have a gearbox reduction of 71
        // PG71 drives 48 teeth gear which drives 40 teeth gear.
        // Quadrature encoder has a pulse per revolution of 7. There are four transitions (low-to-high and high-to-low for A and B signals) per revolution
        // Gear ratio = 40/48 * 71 * 7 * 4 = 1656.666...
        public static final double FL_GEAR_RATIO = -1656.67;
        public static final double FR_GEAR_RATIO = -1656.67;
        public static final double BL_GEAR_RATIO = -1656.67;
        public static final double BR_GEAR_RATIO = -1656.67;

        // Analog encoder positions for each motor controller associated with all motor controllers facing forward.
        // All motors will be facing in the same direction toward the front of the robot.
        // Increasing the analog encoder is counter-clockwise, decreasing is clockwise.
        public static final double FL_TURN_ZERO = 758.0;
        public static final double FR_TURN_ZERO = 807.0;
        public static final double BL_TURN_ZERO = 406.0;
        public static final double BR_TURN_ZERO = 169.0;

        // The actual analog encoder maxmimum positions, since 5V is not supplied to each of the analog encoders.
        public static final int FL_MAX_ANALOG = 886;
        public static final int FR_MAX_ANALOG = 882;
        public static final int BL_MAX_ANALOG = 877;
        public static final int BR_MAX_ANALOG = 880;
    }

    public static final class OI {
        public static enum ControlType {JOYSTICKS, GAMEPAD};
        public static enum StickType {LEFT, RIGHT};
        public static enum StickDirection {X, Y};

        public static ControlType CONTROL_TYPE = ControlType.GAMEPAD;
        public static final double JOY_THRESH = 0.01;
        public static final class LeftJoy {
            public static final int kPort = 0;

            // Joystick buttons
            public static final int fieldOrientedToggle = 2;
            public static final int homeAbsolute = 3;
        }

        public static final class RightJoy {
            public static final int kPort = 1;            
        }

        public static final class Manipulator {        
            public static final int kPort = 2;

            public static final int X = 1;
            public static final int A = 2;
            public static final int B = 3;
            public static final int Y = 4;
            public static final int LB = 5;
            public static final int RB = 6;
            public static final int LT = 7;
            public static final int RT = 8;
            public static final int BACK = 9;
            public static final int START = 10;

            // Joystick buttons
            public static final int fieldOrientedToggle = A;
            public static final int homeAbsolute = B;
        }
    }
}
