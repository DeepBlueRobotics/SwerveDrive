package org.team199.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.team199.lib.SwerveMath;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
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
    private WPI_TalonSRX drive, turn;
    private double edgesPerRevolution;
    private double gearRatio, driveModifier, maxSpeed;
    private double targetAngle;
    private int turnZero, maxAnalog;
    private boolean reversed;
    private Timer timer;
    private SimpleMotorFeedforward simpleMotorFeedforward;//never typo

    /**
     * @param type    The type of the swerve module, either FL (Forward-Left), FR (Forward-Right), 
     *                BL (Backward-Left), or BR (Backward-Right).
     * @param drive   The TalonSRX motor controller for driving forward and backwards.
     * @param turn    The TalonSRX motor controller for turning the module into the correct orientation.
     * @param gearRatio     The gear ratio for the <i> turn </i> motor controller.
     *                      Used for determining the angle of the module.
     * @param driveModifier     A double which controls the speed passed into drive.setSpeed()
     * @param maxSpeed          The maximum speed, in m/s, of the SwerveModule. This should be the same speed when normalizing.
     * @param reversed          Whether or not to reverse the turn motor controller.
     * @param turnZero          The desired raw analog encoder value to reach during HomeAbsolute.
     * @param maxAnalog         The maximum value of the raw analog encoder.
     */
    public SwerveModule(ModuleType type, WPI_TalonSRX drive, WPI_TalonSRX turn, double gearRatio, double driveModifier, 
                        double maxSpeed, boolean reversed, int turnZero, int maxAnalog, double kVolt) {
        this.timer = new Timer();
        timer.start();

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
        catchError(this.drive.configAllowableClosedloopError(0, 4));

        this.turn = turn;
        this.turn.setSensorPhase(true);
        catchError(this.turn.configAllowableClosedloopError(0, 4));

        this.gearRatio = gearRatio;
        this.driveModifier = driveModifier;
        this.maxSpeed = maxSpeed;
        this.reversed = reversed;
        this.turnZero = turnZero;
        this.maxAnalog = maxAnalog;

        catchError(drive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder));
        catchError(turn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder));

        // For an am-3314a CIMcoder, 20 pulses per revolution per channel, 2 edges per pulse = 80 total edges
        edgesPerRevolution = 80.0;

        this.simpleMotorFeedforward = new SimpleMotorFeedforward(kVolt, driveModifier / maxSpeed);
    }

    /**
     * Move the module to a specified angle and drive at a specified speed.
     * @param normalizedSpeed   The desired speed normalized with respect to a maximum speed, in m/s.
     * @param angle             The desired angle, in radians.
     */
    public void move(double normalizedSpeed, double angle) {
        double setpoints[] = SwerveMath.computeSetpoints(normalizedSpeed / maxSpeed,
                                                         -angle / (2 * Math.PI),
                                                         turn.getSelectedSensorPosition(0),
                                                         gearRatio);
        setSpeed(setpoints[0]);
        if(setpoints[0] != 0.0) setAngle(setpoints[1]);
    }

    /**
     * Sets the speed for the drive motor controller.
     * @param speed     The desired speed, from -1.0 (maximum speed directed backwards) to 1.0 (maximum speed directed forwards).
     */
    private void setSpeed(double speed) {
        double deltaTime = timer.get();

        double desiredSpeed = maxSpeed * speed * Math.abs(driveModifier);

        // Calculate acceleration and limit it if greater than maximum acceleration (without slippage and with sufficient motors).
        double desiredAcceleration = (desiredSpeed - getCurrentSpeed()) / deltaTime;
        double maxAcceleration = Constants.DriveConstants.mu * 9.8;
        double clippedAcceleration = Math.copySign(Math.min(Math.abs(desiredAcceleration), maxAcceleration), desiredAcceleration);
        
        double clippedDesiredSpeed = getCurrentSpeed() + clippedAcceleration * deltaTime;
        double appliedVoltage = simpleMotorFeedforward.calculate(clippedDesiredSpeed); //percent of system (12v)

        // Reset the timer so get() returns a change in time
        timer.reset();
        timer.start();
        SmartDashboard.putNumber(moduleString + " Expected Speed", desiredSpeed);
        SmartDashboard.putNumber(moduleString + " Clipped Acceleration", clippedAcceleration);
        SmartDashboard.putNumber(moduleString + " Clipped Speed", clippedDesiredSpeed);
        
        drive.set(ControlMode.PercentOutput, appliedVoltage);
        //drive.set(ControlMode.PercentOutput, Math.copySign(desiredSpeed, desiredSpeed * driveModifier) / maxSpeed);
    }

    /**
     * Sets the angle for the turn motor controller.
     * @param angle     The desired angle, between -0.5 (180 degrees counterclockwise) and 0.5 (180 degrees clockwise).
     */
    private void setAngle(double angle) {
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
        catchError(drive.config_kP(0, kP));
        catchError(drive.config_kI(0, kI));
        catchError(drive.config_kD(0, kD));
    }

    /**
     * Sets the PID constants for the turn motor controller.
     * @param kP        The proportionality constant for the "P" (proportional) term.
     * @param kI        The proportionality constant for the "I" (integral) term.
     * @param kD        The proportionality constant for the "D" (derivative) term.
     */
    public void setTurnPID(double kP, double kI, double kD) {
        catchError(turn.config_kP(0, kP));
        catchError(turn.config_kI(0, kI));
        catchError(turn.config_kD(0, kD));
    }

    /** 
     * Returns the angle of the turn motor controller relative to TURN_ZERO.
     * @return The angle, in radians, of the swerve module.
    */
    private double getModuleAngle() {
        return 2 * Math.PI * ((turn.getSelectedSensorPosition(0) / Math.abs(gearRatio)) % 1);
    }

    /**
     * Gets the current state (speed and angle) of this module.
     * @return A SwerveModuleState object representing the speed and angle of the module.
     */
    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getCurrentSpeed(), new Rotation2d(getModuleAngle()));
    }

    public double getCurrentSpeed() {
        // Encoder edges/sec * revolutions/edge * meters/revolution
        double currentSpeed = drive.getSelectedSensorVelocity(0) * (Math.PI * Constants.DriveConstants.wheelDiameter / edgesPerRevolution);
        currentSpeed /= Constants.DriveConstants.driveGearing;
        // Multiply by 10 since the units of getSelectedSensorVelocity() are in meters per 100 ms
        currentSpeed *= 10;
        return currentSpeed;
    }

    /**
     * Updates SmartDashboard with information about this module.
     */
    public void updateSmartDashboard() {
        // Display the angle that the module is trying to reach.
        SmartDashboard.putNumber(moduleString + " Target Angle", targetAngle);
        // Display the position of the quadrature encoder.
        SmartDashboard.putNumber(moduleString + " Quadrature Position", turn.getSensorCollection().getQuadraturePosition());
        // Display the position of the analog encoder.
        SmartDashboard.putNumber(moduleString + " Analog Position", turn.getSensorCollection().getAnalogIn());
        // Display the raw position of the analog encoder.
        SmartDashboard.putNumber(moduleString + " Raw Analog Position", turn.getSensorCollection().getAnalogInRaw());
        // Display the module angle as calculated using the absolute encoder.
        SmartDashboard.putNumber(moduleString + " Module Angle", getModuleAngle());
        // Display the speed that the robot thinks it is travelling at.
        SmartDashboard.putNumber(moduleString + " Current Speed", getCurrentSpeed());
        // Display the applied voltage to the drive motor.
        SmartDashboard.putNumber(moduleString + " Applied Voltage", drive.getMotorOutputVoltage());
        // Display the output current of the drive motor.
        SmartDashboard.putNumber(moduleString + " Stator Current", drive.getStatorCurrent());
        /////////////////:
        SmartDashboard.putNumber(moduleString + " Drive Encoder Position", drive.getSelectedSensorPosition(0) * (Math.PI * Constants.DriveConstants.wheelDiameter / edgesPerRevolution) / Constants.DriveConstants.driveGearing);
    }

    /**
     * HomeAbsolute is an instant command that ensures that each of the turn motor controllers are in a known configuration,
     * as dictated by the absolute encoder positions turnZero.
     */
    public void homeAbsolute() {
        // The quadrature encoders are for turning the steer motor.
        // The analog encoders are for checking if the motors are in the right position.
        catchError(turn.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder));

        // Change the current quadrature encoder position to the difference between the zeroed position and the current position, as measured by the analog encoder.
        // Difference is in analog encoder degrees which must be converted to quadrature encoder ticks.
        // Max value of the analog encoder is MAX_ANALOG, min value is 0.
        int quadPos = (int) ((Math.abs(gearRatio) / maxAnalog) * (turn.getSensorCollection().getAnalogInRaw() - turnZero));
        
        // Set the orientation of the modules to what they would be relative to TURN_ZERO.
        catchError(turn.setSelectedSensorPosition(quadPos));

        // Make sure we actually turn to the correct position.
        setAngle(0.0);
    }
    
    private void catchError(ErrorCode e) {
        if (e != ErrorCode.OK) {
            System.err.println("Received error code #" + e.value + " for module " + moduleString + " at line " + Thread.currentThread().getStackTrace()[2].getLineNumber() + ".");
        }
    }
}
