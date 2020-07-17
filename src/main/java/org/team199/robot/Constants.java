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

        // maxForward and maxStrafe are in m/s, maxRCW is in radians.
        public static final double maxForward = 0.0;
        public static final double maxStrafe = 0.0;
        public static final double maxRCW = Math.PI;

        // kP, kI, and kD constants for PID Controllers in the order of front-left, front-right, back-left, back-right.
        public static final double[] kP = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kI = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kD = {0.0, 0.0, 0.0, 0.0};

        // Drivetrain Characterization constants in the order of front-left, front-right, back-left, back-right.
        public static final double[] kVels = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kAccels = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kVolts = {0.0, 0.0, 0.0, 0.0};

        // All of the gearboxes are PG71. PG71 have a gearbox reduction of 71
        // PG71 drives 48 teeth gear which drives 40 teeth gear.
        // Quadrature encoder has a pulse per revolution of 7. There are four transitions (low-to-high and high-to-low for A and B signals) per revolution
        // Gear ratio = 40/48 * 71 * 7 * 4 = 1656.666...
        public static final double kFL_GEAR_RATIO = -1656.67;
        public static final double kFR_GEAR_RATIO = -1656.67;
        public static final double kBL_GEAR_RATIO = -1656.67;
        public static final double kBR_GEAR_RATIO = -1656.67;

        // Analog encoder positions for each motor controller associated with all motor controllers facing forward.
        // All motors will be facing in the same direction toward the front of the robot.
        public static final int kFL_TURN_ZERO = 758;
        public static final int kFR_TURN_ZERO = 807;
        public static final int kBL_TURN_ZERO = 406;
        public static final int kBR_TURN_ZERO = 2217;
    }

    public static final class OI {
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
