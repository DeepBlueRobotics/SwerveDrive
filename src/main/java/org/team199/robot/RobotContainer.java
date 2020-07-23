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
import edu.wpi.first.wpilibj.buttons.JoystickButton;

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
      rightJoy = new Joystick(Constants.OI.LeftJoy.kPort);

      if (DriverStation.getInstance().getJoystickName(0).length() != 0) {
        configureButtonBindingsLeftJoy();
      } else {
        System.err.println("ERROR: Left Joystick missing. Perhaps it was not plugged in correctly?");
      }
  
      if (DriverStation.getInstance().getJoystickName(1).length() != 0) {
        configureButtonBindingsRightJoy();
      } else {
        System.err.println("ERROR: Right Joystick missing. Perhaps it was not plugged in correctly?");
      }
      configureButtonBindingsLeftJoy();
      configureButtonBindingsRightJoy();

      drivetrain.setDefaultCommand(new Drive(drivetrain, leftJoy, rightJoy));
      break;
    case GAMEPAD:
      gamepad = new Joystick(Constants.OI.LeftJoy.kPort);
      configureButtonBindingsGamepad();
      drivetrain.setDefaultCommand(new Drive(drivetrain, gamepad));
      break;
    default:
      break;
    }
  }

  private void configureButtonBindingsLeftJoy() {
    homeAbsolute = new JoystickButton(leftJoy, 3);
    homeAbsolute.whenPressed(new HomeAbsolute(drivetrain));
  }

  private void configureButtonBindingsRightJoy() {
  }

  private void configureButtonBindingsGamepad() {
    homeAbsolute = new JoystickButton(gamepad, 3);
    homeAbsolute.whenPressed(new HomeAbsolute(drivetrain));
  }

}
