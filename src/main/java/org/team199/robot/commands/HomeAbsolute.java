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
    double quadPos;
  
    public HomeAbsolute(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
  
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        System.out.println("Home of the abs.");
        SmartDashboard.putNumber("ABSOLUTE count", SmartDashboard.getNumber("ABSOLUTE count", 0) + 1);

        for (int i = 0; i < 4; i++) {
            // The quadrature encoders are for turning the steer motor.
            // The analog encoders are for checking if the motors are in the right position.
            drivetrain.modules[i].changeSelectedSensor(FeedbackDevice.QuadEncoder);

            // Change the current quadrature encoder position to the difference between the zeroed position and the current position, as measured by the analog encoder.
            // Difference is in analog encoder degrees which must be converted to quadrature encoder ticks.
            // Max value of the analog encoder is MAX_ANALOG, min value is 0.
            quadPos = (Math.abs(Constants.DriveConstants.GEAR_RATIO[i]) / Constants.DriveConstants.MAX_ANALOG[i]) * 
                        (drivetrain.modules[i].getAnalogPositionRaw() - Constants.DriveConstants.TURN_ZERO[i]);
            
            // Set the orientation of the modules to what they would be relative to TURN_ZERO.
            drivetrain.modules[i].setSensorPosition((int) quadPos);

            // Make sure we actually turn to the correct position.
            drivetrain.modules[i].setAngle(0.0);
        }
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
  }
  