package org.team199.robot.commands;

import org.team199.robot.Constants;
import org.team199.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The default command to be run on the Drivetrain subsystem. Drives the robot using joysticks.
 */
public class Drive extends CommandBase {
    private Drivetrain drivetrain;
    private Joystick leftJoy;
    private Joystick rightJoy;
    private Joystick gamepad;

    public Drive(Drivetrain drivetrain, Joystick leftJoy, Joystick rightJoy) {
        addRequirements(this.drivetrain = drivetrain);
        this.leftJoy = leftJoy;
        this.rightJoy = rightJoy;
    }

    public Drive(Drivetrain drivetrain, Joystick gamepad) {
        addRequirements(this.drivetrain = drivetrain);
        this.gamepad = gamepad;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forward, strafe, rotateClockwise;
        if (Constants.OI.CONTROL_TYPE == Constants.OI.ControlType.JOYSTICKS) {
            // Inputs to drive are m/s, so joysticks are percentage values of the maximum forward, strafe, and rcw.
            if (Math.abs(leftJoy.getY()) < Constants.OI.LEFT_Y_THRESHOLD) forward = 0.0;
            else forward = Constants.DriveConstants.maxForward * -leftJoy.getY();     // Left joy Y is inverted.
            strafe = Constants.DriveConstants.maxStrafe * leftJoy.getX();
            rotateClockwise = Constants.DriveConstants.maxRCW * rightJoy.getX();
        } else {
            // Inputs to drive are m/s, so joysticks are percentage values of the maximum forward, strafe, and rcw.
            if (Math.abs(gamepad.getRawAxis(1)) < Constants.OI.LEFT_Y_THRESHOLD) forward = 0.0;
            else forward = Constants.DriveConstants.maxForward * -gamepad.getRawAxis(1);     // Left joy Y is inverted.
            strafe = Constants.DriveConstants.maxStrafe * gamepad.getRawAxis(0);
            rotateClockwise = Constants.DriveConstants.maxRCW * gamepad.getRawAxis(2);
            drivetrain.drive(forward, strafe, rotateClockwise);
        }
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