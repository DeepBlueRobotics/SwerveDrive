package org.team199.robot.commands;

import org.team199.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;


/**
 * HomeAbsolute is an instant command that ensures that each of the turn motor controllers are in a known configuration.
 */
public class HomeAbsolute extends InstantCommand {
    Drivetrain drivetrain;
  
    public HomeAbsolute(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
  
    // Called just before this Command runs the first time
    @Override
    public void initialize() {
        drivetrain.homeAbsolute();
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
  }
  