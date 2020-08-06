package org.team199.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.team199.lib.SwerveMath;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    // FL = Forward-Left, FR = Forward-Right, BL = Backward-Left, BR = Backward-Right
    public enum ModuleType {FL, FR, BL, BR};

    private ModuleType type;
    private String moduleString;
    private WPI_TalonSRX drive;
    private WPI_TalonSRX turn;
    private double gearRatio;
    public double targetAngle;
    private double expectedSpeed;

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

    public void move(double normalizedSpeed, double angle, double maxSpeed, double driveModifier, boolean reversed) {
        double setpoints[] = SwerveMath.computeSetpoints(normalizedSpeed / maxSpeed,
                                                         -angle / (2 * Math.PI),
                                                         getSensorPosition(),
                                                         gearRatio);
        System.out.println("Move Values: " + setpoints[0] + ", " + setpoints[1]);
        setSpeed(setpoints[0], maxSpeed, driveModifier);
        if(setpoints[0] != 0.0) setAngle(setpoints[1], reversed);
    }

    public void setSpeed(double speed, double maxSpeed, double driveModifier) {
        // There are no encoders on the drive motor controllers so assume current speed = expected speed
        expectedSpeed = maxSpeed * speed * driveModifier;
        drive.set(ControlMode.PercentOutput, speed * driveModifier);
    }

    // angle must be between -0.5 and 0.5
    public void setAngle(double angle, boolean reversed) {
        targetAngle = (reversed ? -1 : 1) * angle * gearRatio;
        turn.set(ControlMode.Position, targetAngle);
    }

    public void setDrivePID(double kP, double kI, double kD) {
        drive.config_kP(0, kP);
        drive.config_kI(0, kI);
        drive.config_kD(0, kD);
    }

    public void setTurnPID(double kP, double kI, double kD) {
        turn.config_kP(0, kP);
        turn.config_kI(0, kI);
        turn.config_kD(0, kD);
    }

    /** 
     * Returns the position of the quadrature encoder for a turn motor controller.
     * @return The position of the quadrature encoder.
     */
    public int getQuadraturePosition() { return turn.getSensorCollection().getQuadraturePosition(); }

    /** 
     * Returns the velocity of the quadrature encoder for a turn motor controller.
     * @return The velocity of the quadrature encoder.
     */
    public int getQuadratureVelocity() { return turn.getSensorCollection().getQuadratureVelocity(); }

    /** 
     * Returns the position of the analog encoder for a turn motor controller.
     * @return The position of the analog encoder.
     */
    public int getAnalogPosition() { return turn.getSensorCollection().getAnalogIn(); }

    /** 
     * Returns the position of the analog encoder for a turn motor controller.
     * @return The raw position of the analog encoder.
     */
    public int getAnalogPositionRaw() { return turn.getSensorCollection().getAnalogInRaw(); }

    /** 
     * Returns the angle of the analog encoder for a turn motor controller.
     * @return The angle, in radians, of the swerve module.
    */
    public double getModuleAngle(double gearRatio) {
        return 2 * Math.PI * (getSensorPosition() / gearRatio) % 1;
    }

    public int getSensorPosition(){
        return turn.getSelectedSensorPosition(0);
    }

    public void setSensorPosition(int pos) { turn.setSelectedSensorPosition(pos); }

    public void changeSelectedSensor(FeedbackDevice sensor) { turn.configSelectedFeedbackSensor(sensor); }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(expectedSpeed, new Rotation2d(getModuleAngle(gearRatio)));
    }

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