/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot;

import org.team199.robot.commands.HomeAbsolute;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;

  // This function is run when the robot is first started up and should be used for any initialization code.
  @Override
  public void robotInit() {
    SmartDashboard.putBoolean("Field Oriented", true);
    robotContainer = new RobotContainer();
    // Ensure that HomeAbsolute is called on startup.
    CommandScheduler.getInstance().schedule(new HomeAbsolute(robotContainer.drivetrain));
  }

  // This function is called every robot packet, no matter the mode.
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    //robotContainer.homeAbsolutePressed();
  }

  // This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  // This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
  @Override
  public void autonomousInit() {
  }

  // This function is called periodically during autonomous.
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  // This function is called periodically during operator control.
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  // This function is called periodically during test mode.
  @Override
  public void testPeriodic() {
  }
}
