package org.team199.robot.commands;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.team199.robot.Constants;
import org.team199.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
    }
  
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        System.out.println("Home of the abs.");
        SmartDashboard.putNumber("ABSOLUTE count", SmartDashboard.getNumber("ABSOLUTE count", 0) + 1);
        // The quadrature encoders are for turning the steer motor.
        // The analog encoders are for checking if the motors are in the right position.
        drivetrain.moduleFL.changeSelectedSensor(FeedbackDevice.QuadEncoder);
        drivetrain.moduleFR.changeSelectedSensor(FeedbackDevice.QuadEncoder);
        drivetrain.moduleBL.changeSelectedSensor(FeedbackDevice.QuadEncoder);
        drivetrain.moduleBR.changeSelectedSensor(FeedbackDevice.QuadEncoder);

        // Change the current quadrature encoder position to the difference between the zeroed position and the current position, as measured by the analog encoder.
        // Difference is in analog encoder degrees which must be converted to quadrature encoder ticks.
        // Max value of the analog encoder is MAX_ANALOG, min value is 0.
        flQuadPos = (Math.abs(Constants.DriveConstants.FL_GEAR_RATIO) / Constants.DriveConstants.FL_MAX_ANALOG) * 
                    (drivetrain.moduleFL.getAnalogPositionRaw() - Constants.DriveConstants.FL_TURN_ZERO);
        frQuadPos = (Math.abs(Constants.DriveConstants.FR_GEAR_RATIO) / Constants.DriveConstants.FR_MAX_ANALOG) * 
                    (drivetrain.moduleFR.getAnalogPositionRaw() - Constants.DriveConstants.FR_TURN_ZERO);
        blQuadPos = (Math.abs(Constants.DriveConstants.BL_GEAR_RATIO) / Constants.DriveConstants.BL_MAX_ANALOG) * 
                    (drivetrain.moduleBL.getAnalogPositionRaw() - Constants.DriveConstants.BL_TURN_ZERO);
        brQuadPos = (Math.abs(Constants.DriveConstants.BR_GEAR_RATIO) / Constants.DriveConstants.BR_MAX_ANALOG) * 
                    (drivetrain.moduleBR.getAnalogPositionRaw() - Constants.DriveConstants.BR_TURN_ZERO);
        
        drivetrain.moduleFL.setSensorPosition((int) flQuadPos);
        drivetrain.moduleFR.setSensorPosition((int) frQuadPos);
        drivetrain.moduleBL.setSensorPosition((int) blQuadPos);
        drivetrain.moduleBR.setSensorPosition((int) brQuadPos);

        drivetrain.moduleFL.setAngle(0.0, false);
        drivetrain.moduleFR.setAngle(0.0, false);
        drivetrain.moduleBL.setAngle(0.0, false);
        drivetrain.moduleBR.setAngle(0.0, false);
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
  }
  