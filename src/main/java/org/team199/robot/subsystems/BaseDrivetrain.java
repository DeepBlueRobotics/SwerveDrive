package org.team199.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.team199.lib.MotorControllerFactory;
import org.team199.robot.Constants;
import org.team199.robot.SwerveModule;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class BaseDrivetrain extends SubsystemBase {
    protected final AHRS gyro = new AHRS(SerialPort.Port.kUSB1);
    protected final boolean isGyroReversed = true;

    // SwerveModules for each "side" of the robot.
    protected SwerveModule modules[];

    /**
     * Constructs a BaseDrivetrain object
    */
    public BaseDrivetrain() {
        // Initialize SwerveModules
        // Forward-Left

        // Reset gyro before odometry is declared
        gyro.reset();

        SwerveModule moduleFL = new SwerveModule(SwerveModule.ModuleType.FL,
                                    MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontLeft), 
                                    MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontLeft), 
                                    Constants.DriveConstants.driveGearing, Constants.DriveConstants.wheelDiameter,
                                    Constants.DriveConstants.GEAR_RATIO[0], -Constants.DriveConstants.kDriveModifier, 
                                    Constants.DriveConstants.maxSpeed, Constants.DriveConstants.reversed[0],
                                    Constants.DriveConstants.TURN_ZERO[0], Constants.DriveConstants.MAX_ANALOG[0]);
        // Forward-Right
        SwerveModule moduleFR = new SwerveModule(SwerveModule.ModuleType.FR,
                                    MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontRight), 
                                    MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontRight),
                                    Constants.DriveConstants.driveGearing, Constants.DriveConstants.wheelDiameter,
                                    Constants.DriveConstants.GEAR_RATIO[1], Constants.DriveConstants.kDriveModifier,
                                    Constants.DriveConstants.maxSpeed, Constants.DriveConstants.reversed[1],
                                    Constants.DriveConstants.TURN_ZERO[1], Constants.DriveConstants.MAX_ANALOG[1]);
        // Backward-Left
        SwerveModule moduleBL = new SwerveModule(SwerveModule.ModuleType.BL,
                                    MotorControllerFactory.createTalon(Constants.Ports.kDriveBackLeft), 
                                    MotorControllerFactory.createTalon(Constants.Ports.kTurnBackLeft),
                                    Constants.DriveConstants.driveGearing, Constants.DriveConstants.wheelDiameter,
                                    Constants.DriveConstants.GEAR_RATIO[2], -Constants.DriveConstants.kDriveModifier,
                                    Constants.DriveConstants.maxSpeed, Constants.DriveConstants.reversed[2],
                                    Constants.DriveConstants.TURN_ZERO[2], Constants.DriveConstants.MAX_ANALOG[2]);
        // Backward-Right
        SwerveModule moduleBR = new SwerveModule(SwerveModule.ModuleType.BR,
                                    MotorControllerFactory.createTalon(Constants.Ports.kDriveBackRight), 
                                    MotorControllerFactory.createTalon(Constants.Ports.kTurnBackRight),
                                    Constants.DriveConstants.driveGearing, Constants.DriveConstants.wheelDiameter,
                                    Constants.DriveConstants.GEAR_RATIO[3], Constants.DriveConstants.kDriveModifier,
                                    Constants.DriveConstants.maxSpeed, Constants.DriveConstants.reversed[3],
                                    Constants.DriveConstants.TURN_ZERO[3], Constants.DriveConstants.MAX_ANALOG[3]);
        modules = new SwerveModule[]{moduleFL, moduleFR, moduleBL, moduleBR};

        // Configure PID control constants for drive motor controllers
        for (int i = 0; i < 4; i++) {
            modules[i].setDrivePID(Constants.DriveConstants.driveKP[i], Constants.DriveConstants.driveKI[i], Constants.DriveConstants.driveKD[i]);
            modules[i].setTurnPID(Constants.DriveConstants.turnKP[i], Constants.DriveConstants.turnKI[i], Constants.DriveConstants.turnKD[i]);
        }
    }

    /**
     * Periodic method of the Drivetrain. Called every 20 miliseconds.
     */
    @Override
    public void periodic() {
        updateOdometry();
        updateSmartDashboard();
    }

    /** 
     * Returns robot heading from the gyro in degrees.
     */
    protected double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (isGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Runs homeAbsolute for all of the swerve modules.
     */
    public void homeAbsolute() {
        gyro.reset();

        for (int i = 0; i < 4; i++) {
            modules[i].homeAbsolute();
        }
    }

    /**
     * Updates the odometry of the robot.
    */
    protected abstract void updateOdometry();

    /**
     * Updates SmartDashboard with information about the drivetrain pertinent to the operator.
     */
    protected abstract void updateSmartDashboard();

    /**
     * Drives the robot.
     * @param forward   Desired forward speed (in m/s).
     * @param strafe    Desired strafe speed (in m/s).
     * @param rotation  Desired rotation speed (in rad/s).
     */
    public abstract void drive(double forward, double strafe, double rotation);
}
