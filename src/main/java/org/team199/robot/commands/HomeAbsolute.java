package org.team199.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team199.robot.Constants;
import org.team199.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.InstantCommand;


/**
 * HomeAbsolute is an instant command that ensures that each of the turn motor controllers are in a known configuration,
 * as dictated by the absolute encoder positions FL_TURN_ZERO, FR_TURN_ZERO, BL_TURN_ZERO, and BR_TURN_ZERO in Constants.
 */
public class HomeAbsolute extends InstantCommand {
    Drivetrain drivetrain;
    double flQuadPos;
    double frQuadPos;
    double blQuadPos;
    double brQuadPos;
  
    public HomeAbsolute(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        setRunWhenDisabled(true);
    }
  
    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        // The quadrature encoders are for turning the steer motor.
        // The analog encoders are for checking if the motors are in the right position.
        drivetrain.turnFL.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        drivetrain.turnFR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        drivetrain.turnBL.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        drivetrain.turnBR.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Change the current quadrature encoder position to the difference between the zeroed position and the current position, as measured by the analog encoder.
        // Difference is in analog encoder degrees which must be converted to quadrature encoder ticks.
        // Max value of the analog encoder is 1023, min value is 0.
        flQuadPos = (Math.abs(Constants.DriveConstants.FL_GEAR_RATIO) / Constants.DriveConstants.FL_MAX_ANALOG) * 
                    (drivetrain.turnFL.getSensorCollection().getAnalogInRaw() - Constants.DriveConstants.FL_TURN_ZERO);
        frQuadPos = (Math.abs(Constants.DriveConstants.FR_GEAR_RATIO) / Constants.DriveConstants.FR_MAX_ANALOG) * 
                    (drivetrain.turnFR.getSensorCollection().getAnalogInRaw() - Constants.DriveConstants.FR_TURN_ZERO);
        blQuadPos = (Math.abs(Constants.DriveConstants.BL_GEAR_RATIO) / Constants.DriveConstants.BL_MAX_ANALOG) * 
                    (drivetrain.turnBL.getSensorCollection().getAnalogInRaw() - Constants.DriveConstants.BL_TURN_ZERO);
        brQuadPos = (Math.abs(Constants.DriveConstants.BR_GEAR_RATIO) / Constants.DriveConstants.BR_MAX_ANALOG) * 
                    (drivetrain.turnBR.getSensorCollection().getAnalogInRaw() - Constants.DriveConstants.BR_TURN_ZERO);
        
        drivetrain.turnFL.setSelectedSensorPosition((int) flQuadPos);
        drivetrain.turnFR.setSelectedSensorPosition((int) frQuadPos);
        drivetrain.turnBL.setSelectedSensorPosition((int) blQuadPos);
        drivetrain.turnBR.setSelectedSensorPosition((int) brQuadPos);

        drivetrain.turnFL.set(ControlMode.Position, 0.0);
        drivetrain.turnFR.set(ControlMode.Position, 0.0);
        drivetrain.turnBR.set(ControlMode.Position, 0.0);
        drivetrain.turnBL.set(ControlMode.Position, 0.0);
    }
  }
  