/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot;

import org.team199.robot.commands.Drive;
import org.team199.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {
  private final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.kPort);
  private final Joystick rightJoy = new Joystick(Constants.OI.LeftJoy.kPort);
  private final Joystick manipulator = new Joystick(Constants.OI.LeftJoy.kPort);

  private final Drivetrain drivetrain = new Drivetrain(Drivetrain.SwerveImplementation.WPILib);

  public RobotContainer() {
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

    if (DriverStation.getInstance().getJoystickName(2).length() != 0) {
      configureButtonBindingsManipulator();
    } else {
      System.err.println("ERROR: Manipulator (Controller) missing. Perhaps it was not plugged in correctly?");
    }

    drivetrain.setDefaultCommand(new Drive(drivetrain, leftJoy, rightJoy));
  }

  private void configureButtonBindingsLeftJoy() {
  }

  private void configureButtonBindingsRightJoy() {
  }

  private void configureButtonBindingsManipulator() {
  }
}
