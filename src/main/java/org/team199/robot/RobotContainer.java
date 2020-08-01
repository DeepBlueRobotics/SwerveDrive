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

  private final Drivetrain drivetrain = new Drivetrain(Drivetrain.SwerveImplementation.WPILib);

  public RobotContainer() {
    switch (Constants.OI.CONTROL_TYPE) {
      case JOYSTICKS:
        leftJoy = new Joystick(Constants.OI.LeftJoy.kPort);
        rightJoy = new Joystick(Constants.OI.RightJoy.kPort);

        if (DriverStation.getInstance().getJoystickName(Constants.OI.LeftJoy.kPort).length() != 0) {
          configureButtonBindingsLeftJoy();

          if (DriverStation.getInstance().getJoystickName(Constants.OI.RightJoy.kPort).length() != 0) {
            configureButtonBindingsRightJoy();
            drivetrain.setDefaultCommand(new Drive(drivetrain, leftJoy, rightJoy));
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
          drivetrain.setDefaultCommand(new Drive(drivetrain, gamepad));
        } else {
          System.err.println("ERROR: Manipulator missing. Perhaps it was not plugged in correctly?");
        }
        break;
      }
  }

  private void configureButtonBindingsLeftJoy() {
    homeAbsolute = new JoystickButton(leftJoy, Constants.OI.LeftJoy.homeAbsolute);
    homeAbsolute.whenPressed(new HomeAbsolute(drivetrain));

    // Toggle whether or not we are in field oriented mode.
    new JoystickButton(leftJoy, Constants.OI.LeftJoy.fieldOrientedToggle).whenPressed(new InstantCommand(() -> {
      SmartDashboard.putBoolean("Field Oriented", !SmartDashboard.getBoolean("Field Oriented", true));
    }));
  }

  private void configureButtonBindingsRightJoy() {
  }

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
}
