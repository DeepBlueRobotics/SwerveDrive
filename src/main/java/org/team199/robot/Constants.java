/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot;

// TODO: All constants must be updated to their true values.
public final class Constants {
    public static final class Ports {
        // CAN ids for each of the swerve-drive motor controllers.
        public static final int kDriveFrontLeft = 0;
        public static final int kDriveFrontRight = 15;
        public static final int kDriveBackLeft = 1;
        public static final int kDriveBackRight = 14;
        
        public static final int kTurnFrontLeft = 4;
        public static final int kTurnFrontRight = 11;
        public static final int kTurnBackLeft = 5;
        public static final int kTurnBackRight = 10;
    }

    public static final class DriveConstants {
        public static final double wheelBase = 18.15;
        public static final double trackWidth = 17.75;

        // kP, kI, and kD constants for PID Controllers in the order of front-left, front-right, back-left, back-right.
        public static final double[] kP = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kI = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kD = {0.0, 0.0, 0.0, 0.0};

        // Drivetrain Characterization constants in the order of front-left, front-right, back-left, back-right.
        public static final double[] kVels = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kAccels = {0.0, 0.0, 0.0, 0.0};
        public static final double[] kVolts = {0.0, 0.0, 0.0, 0.0};
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
