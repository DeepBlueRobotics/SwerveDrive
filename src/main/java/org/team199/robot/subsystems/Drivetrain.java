/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot.subsystems;

import org.team199.lib.MotorControllerFactory;
import org.team199.lib.SwerveContainer;
import org.team199.lib.SwerveMath;
import org.team199.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  public enum SwerveImplementation {
    Custom, WPILib;
  }

  private final SwerveImplementation implementation;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private final AHRS gyro = new AHRS(SerialPort.Port.kUSB1);
  private final boolean isGyroReversed = true;

  // Replace with actual motor controllers used, if needed. Taken from Team 100's code.
  private WPI_TalonSRX driveFL = MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontLeft);
  private WPI_TalonSRX driveFR = MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontRight);
  private WPI_TalonSRX driveBL = MotorControllerFactory.createTalon(Constants.Ports.kDriveBackLeft);
  private WPI_TalonSRX driveBR = MotorControllerFactory.createTalon(Constants.Ports.kDriveBackRight);

  private WPI_TalonSRX turnFL = MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontLeft);
  private WPI_TalonSRX turnFR = MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontRight);
  private WPI_TalonSRX turnBL = MotorControllerFactory.createTalon(Constants.Ports.kTurnBackLeft);
  private WPI_TalonSRX turnBR = MotorControllerFactory.createTalon(Constants.Ports.kTurnBackRight);

  private double forward = 0.0;
  private double strafe = 0.0;
  private double rotation = 0.0;

  public Drivetrain(SwerveImplementation implementation) {
    this.implementation = implementation;
    gyro.reset();

    // Decide whether or not to use custom implementation of swerve drive or use WPILib implementation using Odometry/Kinematics.
    if (implementation == SwerveImplementation.WPILib) {
      // Define the corners of the robot relative to the center of the robot using Translation2d objects.
      Translation2d locationFL = new Translation2d(-Constants.DriveConstants.frameWidth / 2, Constants.DriveConstants.frameLength / 2);
      Translation2d locationFR = new Translation2d(Constants.DriveConstants.frameWidth / 2, Constants.DriveConstants.frameLength / 2);
      Translation2d locationBL = new Translation2d(-Constants.DriveConstants.frameWidth / 2, -Constants.DriveConstants.frameLength / 2);
      Translation2d locationBR = new Translation2d(Constants.DriveConstants.frameWidth / 2, -Constants.DriveConstants.frameLength / 2);

      kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
      odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Math.toRadians(getHeading())));
    }
  }

  @Override
  public void periodic() {
    if (implementation == SwerveImplementation.WPILib) {
      odometry.update(Rotation2d.fromDegrees(getHeading()), getSwerveStates(forward, strafe, rotation));
    }
  }

  // Returns robot heading from the gyro in degrees.
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (isGyroReversed ? -1.0 : 1.0);
  }

  // Forward and strafe are measured in meters/second, rotation is measured in radians/second.
  public void drive(double forward, double strafe, double rotation) {
    this.forward = forward;
    this.strafe = strafe;
    this.rotation = rotation;

    if (implementation == SwerveImplementation.WPILib) {
      SwerveModuleState[] moduleStates = getSwerveStates(forward, strafe, rotation);
      double[] driveSpeeds = {moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond,
                              moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond};
      driveSpeeds = SwerveMath.normalize(driveSpeeds);

      driveFL.set(driveSpeeds[0]);
      driveFR.set(driveSpeeds[1]);
      driveBL.set(driveSpeeds[2]);
      driveBR.set(driveSpeeds[3]);

      turnFL.set(moduleStates[0].angle.getRadians() / (2 * Math.PI));
      turnFR.set(moduleStates[0].angle.getRadians() / (2 * Math.PI));
      turnBL.set(moduleStates[0].angle.getRadians() / (2 * Math.PI));
      turnBR.set(moduleStates[0].angle.getRadians() / (2 * Math.PI));
    } else {
      SwerveContainer[] containers;
      if (SmartDashboard.getBoolean("Field Oriented", true)) {
        containers = SwerveMath.calculateSwerve(forward, strafe, rotation, Math.toRadians(getHeading()), 
                                                Constants.DriveConstants.frameLength, Constants.DriveConstants.frameWidth);
      } else {
        containers = SwerveMath.calculateSwerve(forward, strafe, rotation,
                                                Constants.DriveConstants.frameLength, Constants.DriveConstants.frameWidth);
      }

      driveFL.set(containers[0].speed);
      driveFR.set(containers[1].speed);
      driveBL.set(containers[2].speed);
      driveBR.set(containers[3].speed);

      turnFL.set(containers[0].angle / (2 * Math.PI));
      turnFR.set(containers[1].angle / (2 * Math.PI));
      turnBL.set(containers[2].angle / (2 * Math.PI));
      turnBR.set(containers[3].angle / (2 * Math.PI));
    }
  }

  private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds;
    if (SmartDashboard.getBoolean("Field Oriented", true)) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()));
    } else {
      speeds = new ChassisSpeeds(forward, strafe, rotation);
    }
    return kinematics.toSwerveModuleStates(speeds);
  }
}
