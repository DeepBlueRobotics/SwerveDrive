/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot;

public final class Constants {
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
        public static final double wheelBase = 18.15;
        public static final double trackWidth = 17.75;

        // maxForward and maxStrafe are in m/s, maxRCW is in radians, and maxSpeed is in m/s.
        // TODO: Find max values.
        public static final double maxForward = 0.0;
        public static final double maxStrafe = 0.0;
        public static final double maxRCW = Math.PI;
        public static final double maxSpeed = 0.0;

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
        public static final boolean reversedFL = true;
        public static final boolean reversedFR = true;
        public static final boolean reversedBL = true;
        public static final boolean reversedBR = true;

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
        public enum ControlType {JOYSTICKS, GAMEPAD};

        public static ControlType CONTROL_TYPE = ControlType.GAMEPAD;
        public static final double LEFT_Y_THRESHOLD = 0.008;
        public static final class LeftJoy {
            public static final int kPort = 0;
        }

        public static final class RightJoy {
            public static final int kPort = 1;            
        }

        public static final class Manipulator {        
            public static final int kPort = 2;
        }
    }
}
