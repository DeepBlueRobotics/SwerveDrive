package org.team199.robot.commands;

import org.team199.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Drive extends CommandBase {
    private Drivetrain drivetrain;
    private Joystick leftJoy;
    private Joystick rightJoy;

    public Drive(Drivetrain drivetrain, Joystick leftJoy, Joystick rightJoy) {
        addRequirements(this.drivetrain = drivetrain);
        this.leftJoy = leftJoy;
        this.rightJoy = rightJoy;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forward = -leftJoy.getY();
        double strafe = leftJoy.getX();
        double rotateClockwise = rightJoy.getX();
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