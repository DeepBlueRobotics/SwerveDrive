package org.team199.robot.commands;

import java.util.function.Supplier;

import org.team199.robot.Constants;
import org.team199.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The default command to be run on the Drivetrain subsystem. Drives the robot using joysticks.
 */
public class Drive extends CommandBase {
    private Drivetrain drivetrain;
    private Supplier<Double> fwd;
    private Supplier<Double> str;
    private Supplier<Double> rcw;

    public Drive(Drivetrain drivetrain, Supplier<Double> fwd, Supplier<Double> str, Supplier<Double> rcw) {
        addRequirements(this.drivetrain = drivetrain);
        this.fwd = fwd;
        this.str = str;
        this.rcw = rcw;
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forward, strafe, rotateClockwise;

        if (Math.abs(fwd.get()) <= Constants.OI.JOY_THRESH) forward = 0.0;
        else forward = Constants.DriveConstants.maxForward * fwd.get();     // Left joy Y is inverted.
        if (Math.abs(str.get()) <= Constants.OI.JOY_THRESH) strafe = 0.0;
        else strafe = Constants.DriveConstants.maxStrafe * str.get();
        if (Math.abs(rcw.get()) <= Constants.OI.JOY_THRESH) rotateClockwise = 0.0;
        else rotateClockwise = Constants.DriveConstants.maxRCW * rcw.get();
        
        drivetrain.drive(forward, strafe, rotateClockwise);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}