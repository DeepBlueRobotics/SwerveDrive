/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot;

import org.team199.robot.commands.Drive;
import org.team199.robot.commands.HomeAbsolute;
import org.team199.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  private Joystick leftJoy;
  private Joystick rightJoy;
  private Joystick gamepad;
  private JoystickButton homeAbsolute;

  public final Drivetrain drivetrain = new Drivetrain(Drivetrain.SwerveImplementation.WPILib);

  public RobotContainer() {
    switch (Constants.OI.CONTROL_TYPE) {
      case JOYSTICKS:
        leftJoy = new Joystick(Constants.OI.LeftJoy.kPort);
        rightJoy = new Joystick(Constants.OI.RightJoy.kPort);

        if (DriverStation.getInstance().getJoystickName(Constants.OI.LeftJoy.kPort).length() != 0) {
          configureButtonBindingsLeftJoy();

          if (DriverStation.getInstance().getJoystickName(Constants.OI.RightJoy.kPort).length() != 0) {
            configureButtonBindingsRightJoy();
          } else {
            System.err.println("ERROR: Right Joystick missing. Perhaps it was not plugged in correctly?");
          }
        } else {
          System.err.println("ERROR: Left Joystick missing. Perhaps it was not plugged in correctly?");
        }
        break;
      case GAMEPAD:
        gamepad = new Joystick(Constants.OI.Manipulator.kPort);

        if (DriverStation.getInstance().getJoystickName(Constants.OI.Manipulator.kPort).length() != 0) {
          configureButtonBindingsGamepad();
        } else {
          System.err.println("ERROR: Manipulator missing. Perhaps it was not plugged in correctly?");
        }
        break;
    }

    drivetrain.setDefaultCommand(new Drive(drivetrain, 
                                 () -> signedSquare(getStickValue(Constants.OI.StickType.LEFT, Constants.OI.StickDirection.Y)),
                                 () -> signedSquare(getStickValue(Constants.OI.StickType.LEFT, Constants.OI.StickDirection.X)),
                                 () -> signedSquare(getStickValue(Constants.OI.StickType.RIGHT, Constants.OI.StickDirection.X))));
  }

  /**
   * Configures buttons on the Left Joystick.
   */
  private void configureButtonBindingsLeftJoy() {
    homeAbsolute = new JoystickButton(leftJoy, Constants.OI.LeftJoy.homeAbsolute);
    homeAbsolute.whenPressed(new HomeAbsolute(drivetrain));

    // Toggle whether or not we are in field oriented mode.
    new JoystickButton(leftJoy, Constants.OI.LeftJoy.fieldOrientedToggle).whenPressed(new InstantCommand(() -> {
      SmartDashboard.putBoolean("Field Oriented", !SmartDashboard.getBoolean("Field Oriented", true));
    }));
  }

  /**
   * Configures buttons on the Right Joystick.
   */
  private void configureButtonBindingsRightJoy() {
  }

  /**
   * Configures buttons on the Gamepad.
   */
  private void configureButtonBindingsGamepad() {
    homeAbsolute = new JoystickButton(gamepad, Constants.OI.Manipulator.homeAbsolute);
    homeAbsolute.whenPressed(new HomeAbsolute(drivetrain));

    System.out.println("Buttons be ok.");

    // Toggle whether or not we are in field oriented mode.
    new JoystickButton(gamepad, Constants.OI.Manipulator.fieldOrientedToggle).whenPressed(new InstantCommand(() -> {
      SmartDashboard.putBoolean("Field Oriented", !SmartDashboard.getBoolean("Field Oriented", true));
    }));
  }

  /*public void homeAbsolutePressed() {
    System.out.println(gamepad.getRawButtonPressed(Constants.OI.Manipulator.homeAbsolute));
  }*/

  /**
   * Get the stick value of a joystick given its stick type (left side or right side) and its axis (X or Y).
   * @param stick   The stick type of the joystick, either LEFT for left joystick or RIGHT for right joystick.
   * @param dir     The direction, or axis, of the joystick, either X for the x-axis or Y for the y-axis.
   * @return A double representing how far the joystick has been pushed, between -1.0 (all the way backwards) to 1.0 (all the way forwards).
   */
  public double getStickValue(Constants.OI.StickType stick, Constants.OI.StickDirection dir) {
    switch (Constants.OI.CONTROL_TYPE) {
      case JOYSTICKS:
        if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X) return leftJoy.getX();
        if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y) return -leftJoy.getY();
        if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X) return rightJoy.getX();
        if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y) return -rightJoy.getY();
      case GAMEPAD:
        if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X) return gamepad.getRawAxis(0);
        if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y) return -gamepad.getRawAxis(1);
        if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X) return gamepad.getRawAxis(2);
        if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y) return -gamepad.getRawAxis(3);
      default: return 0;
    }
  }

  private double signedSquare(double value){
    return value * Math.abs(value);
  }

}
