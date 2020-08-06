package org.team199.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.team199.lib.SwerveMath;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class that stores all the variables and methods applicaple to a single swerve module,
 * such as moving, getting encoder values, or configuring PID.
 */
public class SwerveModule {
    public enum ModuleType {FL, FR, BL, BR};

    private ModuleType type;
    private String moduleString;
    private WPI_TalonSRX drive;
    private WPI_TalonSRX turn;
    private double gearRatio;
    public double targetAngle;
    private double expectedSpeed;

    /**
     * @param type    The type of the swerve module, either FL (Forward-Left), FR (Forward-Right), 
     *                BL (Backward-Left), or BR (Backward-Right).
     * @param drive   The TalonSRX motor controller for driving forward and backwards.
     * @param turn    The TalonSRX motor controller for turning the module into the correct orientation.
     * @param gearRatio     The gear ratio for the <i> turn </i> motor controller.
     *                      Used for determining the angle of the module.
     */
    public SwerveModule(ModuleType type, WPI_TalonSRX drive, WPI_TalonSRX turn, double gearRatio) {
        this.type = type;

        switch (type) {
            case FL:
                moduleString = "FL";
                break;
            case FR:
                moduleString = "FR";
                break;
            case BL:
                moduleString = "BL";
                break;
            case BR:
                moduleString = "BR";
                break;
        }

        this.drive = drive;
        this.drive.setSensorPhase(true);
        this.drive.configAllowableClosedloopError(0, 4);
        this.turn = turn;
        this.turn.setSensorPhase(true);
        this.turn.configAllowableClosedloopError(0, 4);
        this.gearRatio = gearRatio;
        expectedSpeed = 0.0;
        changeSelectedSensor(FeedbackDevice.QuadEncoder);
    }

    /**
     * Move the module to a specified angle and drive at a specified speed.
     * @param normalizedSpeed   The desired speed normalized with respect to a maximum speed, in m/s.
     * @param angle             The desired angle, in radians.
     * @param maxSpeed          The maximum speed at which to drive at, in m/s.
     * @param driveModifier     A constant for controlling how fast the robot drives.
     * @param reversed          A boolean representing whether or not to reverse turning.
     */
    public void move(double normalizedSpeed, double angle, double maxSpeed, double driveModifier, boolean reversed) {
        double setpoints[] = SwerveMath.computeSetpoints(normalizedSpeed / maxSpeed,
                                                         -angle / (2 * Math.PI),
                                                         getSensorPosition(),
                                                         gearRatio);
        System.out.println("Move Values: " + setpoints[0] + ", " + setpoints[1]);
        setSpeed(setpoints[0], maxSpeed, driveModifier);
        if(setpoints[0] != 0.0) setAngle(setpoints[1], reversed);
    }

    /**
     * Sets the speed for the drive motor controller.
     * @param speed     The desired speed, from -1.0 (maximum speed directed backwards) to 1.0 (maximum speed directed forwards).
     * @param maxSpeed  The maximum speed at which to drive at, in m/s^2.
     * @param driveModifier     A constant for controlling how fast the robot drives.
     */
    public void setSpeed(double speed, double maxSpeed, double driveModifier) {
        // There are no encoders on the drive motor controllers so assume current speed = expected speed
        expectedSpeed = maxSpeed * speed * driveModifier;
        drive.set(ControlMode.PercentOutput, speed * driveModifier);
    }

    /**
     * Sets the angle for the turn motor controller.
     * @param angle     The desired angle, between -0.5 (180 degrees counterclockwise) and 0.5 (180 degrees clockwise).
     * @param reversed  A boolean representing whether or not to reverse turning.
     */
    public void setAngle(double angle, boolean reversed) {
        targetAngle = (reversed ? -1 : 1) * angle * gearRatio;
        turn.set(ControlMode.Position, targetAngle);
    }

    /**
     * Sets the PID constants for the drive motor controller.
     * @param kP        The proportionality constant for the "P" (proportional) term.
     * @param kI        The proportionality constant for the "I" (integral) term.
     * @param kD        The proportionality constant for the "D" (derivative) term.
     */
    public void setDrivePID(double kP, double kI, double kD) {
        drive.config_kP(0, kP);
        drive.config_kI(0, kI);
        drive.config_kD(0, kD);
    }

    /**
     * Sets the PID constants for the turn motor controller.
     * @param kP        The proportionality constant for the "P" (proportional) term.
     * @param kI        The proportionality constant for the "I" (integral) term.
     * @param kD        The proportionality constant for the "D" (derivative) term.
     */
    public void setTurnPID(double kP, double kI, double kD) {
        turn.config_kP(0, kP);
        turn.config_kI(0, kI);
        turn.config_kD(0, kD);
    }

    /** 
     * Returns the position of the quadrature encoder for the turn motor controller.
     * @return The position of the quadrature encoder.
     */
    public int getQuadraturePosition() { return turn.getSensorCollection().getQuadraturePosition(); }

    /** 
     * Returns the velocity of the quadrature encoder for the turn motor controller.
     * @return The velocity of the quadrature encoder.
     */
    public int getQuadratureVelocity() { return turn.getSensorCollection().getQuadratureVelocity(); }

    /** 
     * Returns the position of the analog encoder for the turn motor controller.
     * @return The position of the analog encoder.
     */
    public int getAnalogPosition() { return turn.getSensorCollection().getAnalogIn(); }

    /** 
     * Returns the position of the analog encoder for the turn motor controller.
     * @return The raw position of the analog encoder.
     */
    public int getAnalogPositionRaw() { return turn.getSensorCollection().getAnalogInRaw(); }

    /** 
     * Returns the angle of the turn motor controller relative to TURN_ZERO.
     * @return The angle, in radians, of the swerve module.
    */
    public double getModuleAngle(double gearRatio) {
        return 2 * Math.PI * (getSensorPosition() / gearRatio) % 1;
    }

    /**
     * @return The value of the currently selected sensor.
     */
    public int getSensorPosition() {
        return turn.getSelectedSensorPosition(0);
    }

    /**
     * Sets the position of the currently selected sensor to a desired value.
     * @param pos   The position to set the selected sensor's position to.
     */
    public void setSensorPosition(int pos) { 
        turn.setSelectedSensorPosition(pos);
    }

    /**
     * Changes the currently selected sensor.
     * @param sensor      The desired sensor type.
     */
    public void changeSelectedSensor(FeedbackDevice sensor) { 
        turn.configSelectedFeedbackSensor(sensor); 
    }

    /**
     * Gets the current state (speed and angle) of this module.
     * @return A SwerveModuleState object representing the speed and angle of the module.
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(expectedSpeed, new Rotation2d(getModuleAngle(gearRatio)));
    }

    /**
     * Updates SmartDashboard with information about this module.
     */
    public void updateSmartDahsboard() {
        // Display the angle that the module is trying to reach.
        SmartDashboard.putNumber(moduleString + " Target Angle", targetAngle);
        // Display the position of the quadrature encoder.
        SmartDashboard.putNumber(moduleString + " Quadrature Position", getQuadraturePosition());
        // Display the position of the analog encoder.
        SmartDashboard.putNumber(moduleString + " Analog Position", getAnalogPosition());
        // Display the raw position of the analog encoder.
        SmartDashboard.putNumber(moduleString + " Raw Analog Position", getAnalogPositionRaw());
        // Display the module angle as calculated using the absolute encoder.
        SmartDashboard.putNumber(moduleString + " Module Angle", getModuleAngle(gearRatio));
    }
}