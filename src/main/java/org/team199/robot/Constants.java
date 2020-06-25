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
        public static final int kDriveFrontRight = 0;
        public static final int kDriveBackLeft = 0;
        public static final int kDriveBackRight = 0;
        
        public static final int kTurnFrontLeft = 0;
        public static final int kTurnFrontRight = 0;
        public static final int kTurnBackLeft = 0;
        public static final int kTurnBackRight = 0;
    }

    public static final class DriveConstants {
        public static final double frameLength = 0.0;
        public static final double frameWidth = 0.0;

        // Maximum values for forward, strafe, and rotate clockwise values in m/s, m/s, and radians respectively.
        public static final double maxForward = 0.0;
        public static final double maxStrafe = 0.0;
        public static final double maxRotate = 0.0;

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
