package org.team199.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.team199.lib.SwerveMath;

public class SwerveModule {
    private WPI_TalonSRX drive;
    private WPI_TalonSRX turn;
    private double gearRatio;

    public SwerveModule(WPI_TalonSRX drive, WPI_TalonSRX turn, double gearRatio) {
        this.drive = drive;
        this.turn = turn;
        this.gearRatio = gearRatio;
    }

    public void move(double normalizedSpeed, double angle, double driveModifier, boolean reversed) {
        double setpoints[] = SwerveMath.computeSetpoints(normalizedSpeed, 
                                                        angle / (2 * Math.PI),
                                                        this.getQuadraturePosition(),
                                                        gearRatio);
        this.setSpeed(setpoints[0], driveModifier);
        this.setAngle(setpoints[1], reversed);
    }

    public void setSpeed(double speed, double driveModifier) {
        drive.set(ControlMode.PercentOutput, speed * driveModifier);
    }

    public void setAngle(double angle, boolean reversed) {
        turn.set(ControlMode.Position, (reversed ? -1 : 1) * angle * gearRatio);
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
     * Returns the angle of the analog encoder for a turn motor controller.
     * @return The angle, in radians, of the swerve module.
    */
    public double getModuleAngle(int TURN_ZERO, int MAX_ANALOG) {
        // One full revolution is MAX_ANALOG on the analog encoder, so a quarter revolution is (MAX_ANALOG / 4).
        // Therefore zero degrees is located at (TURN_ZERO - MAX_ANALOG / 4) mod MAX_ANALOG (call this reference) since TURN_ZERO points to straight forward (90 degrees)
        // MAX_ANALOG analog = 2 * pi radians so analog to radian is (2 * pi / MAX_ANALOG)(Current analog - reference)
        int reference = (int) (TURN_ZERO - (MAX_ANALOG / 4)) % MAX_ANALOG;
        double angle = ((Math.PI * 2) / MAX_ANALOG) * (turn.getSensorCollection().getAnalogIn() - reference);
        if (angle < 0) return 2 * Math.PI + angle;
        else return angle;
    }

    public void setSensorPosition(int pos) { turn.setSelectedSensorPosition(pos); }

    public void changeSelectedSensor(FeedbackDevice sensor) { turn.configSelectedFeedbackSensor(sensor); }
}