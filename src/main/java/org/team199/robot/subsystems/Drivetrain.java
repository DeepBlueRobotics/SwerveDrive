/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot.subsystems;

import org.team199.lib.MotorControllerFactory;
import org.team199.lib.SwerveMath;
import org.team199.robot.Constants;
import org.team199.robot.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The Drivetrain subsystem. 
 * Contains the majority of methods associated with driving the robot or updating the position of the robot.
 * NOTE: Make sure to run the HomeAbsolute command before you start driving, as some methods rely on the assumption that
 * 0 on the quadrature encoder = straight forward (the Drive() command will not function correctly if you do not call HomeAbsolute).
 */
public class Drivetrain extends SubsystemBase {
  /** Controls which codebase to upon deploy:
   * 
   * <p><ul>
   * <li> Custom - code written by Team 199 from the ground-up (mostly).
   * <li> WPILib - WPILib API that handles the complicated math.
   * </ul><p>
  */
  public enum SwerveImplementation {
    Custom, WPILib;
  }

  private final SwerveImplementation implementation;
  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;

  private final AHRS gyro = new AHRS(SerialPort.Port.kUSB1);
  private final boolean isGyroReversed = true;

  // SwerveModules for each "side" of the robot.
  public SwerveModule moduleFL;
  public SwerveModule moduleFR;
  public SwerveModule moduleBL;
  public SwerveModule moduleBR;

  // Odometry equivalent for non-WPILib implementation
  public Pose2d pose;
  private double gyroOffset;

  /**
   * Constructs a Drivetrain object
   * @param implementation    The chosen implementation type. 
   * NOTE: This cannot be changed during runtime and is updated when code is deployed to the robot.
   */
  public Drivetrain(SwerveImplementation implementation) {
    this.implementation = implementation;
    gyro.reset();
    double heading = Math.toRadians(getHeading());

    // Initialize SwerveModules
    moduleFL = new SwerveModule(SwerveModule.ModuleType.FL,
                                MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontLeft), 
                                MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontLeft), 
                                Constants.DriveConstants.FL_GEAR_RATIO);
    moduleFR = new SwerveModule(SwerveModule.ModuleType.FR,
                                MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontRight), 
                                MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontRight), 
                                Constants.DriveConstants.FR_GEAR_RATIO);
    moduleBL = new SwerveModule(SwerveModule.ModuleType.BL,
                                MotorControllerFactory.createTalon(Constants.Ports.kDriveBackLeft), 
                                MotorControllerFactory.createTalon(Constants.Ports.kTurnBackLeft), 
                                Constants.DriveConstants.BL_GEAR_RATIO);
    moduleBR = new SwerveModule(SwerveModule.ModuleType.BR,
                                MotorControllerFactory.createTalon(Constants.Ports.kDriveBackRight), 
                                MotorControllerFactory.createTalon(Constants.Ports.kTurnBackRight), 
                                Constants.DriveConstants.BR_GEAR_RATIO);

    // Configure PID control constants for drive motor controllers
    moduleFL.setDrivePID(Constants.DriveConstants.driveKP[0], Constants.DriveConstants.driveKI[0], Constants.DriveConstants.driveKD[0]);
    moduleFR.setDrivePID(Constants.DriveConstants.driveKP[1], Constants.DriveConstants.driveKI[1], Constants.DriveConstants.driveKD[1]);
    moduleBL.setDrivePID(Constants.DriveConstants.driveKP[2], Constants.DriveConstants.driveKI[2], Constants.DriveConstants.driveKD[2]);
    moduleBR.setDrivePID(Constants.DriveConstants.driveKP[3], Constants.DriveConstants.driveKI[3], Constants.DriveConstants.driveKD[3]);

    // Configure PID control constants for turn motor controllers
    moduleFL.setTurnPID(Constants.DriveConstants.turnKP[0], Constants.DriveConstants.turnKI[0], Constants.DriveConstants.turnKD[0]);
    moduleFR.setTurnPID(Constants.DriveConstants.turnKP[1], Constants.DriveConstants.turnKI[1], Constants.DriveConstants.turnKD[1]);
    moduleBL.setTurnPID(Constants.DriveConstants.turnKP[2], Constants.DriveConstants.turnKI[2], Constants.DriveConstants.turnKD[2]);
    moduleBR.setTurnPID(Constants.DriveConstants.turnKP[3], Constants.DriveConstants.turnKI[3], Constants.DriveConstants.turnKD[3]);

    // Decide whether or not to use custom implementation of swerve drive or use WPILib implementation using Odometry/Kinematics.
    if (implementation == SwerveImplementation.WPILib) {
      // Define the corners of the robot relative to the center of the robot using Translation2d objects.
      // Positive x-values represent moving toward the front of the robot whereas positive y-values represent moving toward the left of the robot.
      Translation2d locationFL = new Translation2d(Constants.DriveConstants.wheelBase / 2, Constants.DriveConstants.trackWidth / 2);
      Translation2d locationFR = new Translation2d(Constants.DriveConstants.wheelBase / 2, -Constants.DriveConstants.trackWidth / 2);
      Translation2d locationBL = new Translation2d(-Constants.DriveConstants.wheelBase / 2, Constants.DriveConstants.trackWidth / 2);
      Translation2d locationBR = new Translation2d(-Constants.DriveConstants.wheelBase / 2, -Constants.DriveConstants.trackWidth / 2);

      kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
      odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(heading));
    } else {
      Rotation2d rot = new Rotation2d(heading);
      pose = new Pose2d(new Translation2d(), rot);
      gyroOffset = pose.getRotation().minus(rot).getRadians();
    }
  }

  /**
   * Updates the odometry of the robot, either via SwerveDriveOdometry.update() or via the custom implementation.
   * See section 10 of <a href = "https://file.tavsys.net/control/controls-engineering-in-frc.pdf"> Controls Engineering in the
FIRST Robotics Competition </a> by Tyler Veness for more information.
   */
  public void updateOdometry() {
    SwerveModuleState flState = moduleFL.getCurrentState();
    SwerveModuleState frState = moduleFR.getCurrentState();
    SwerveModuleState blState = moduleBL.getCurrentState();
    SwerveModuleState brState = moduleBR.getCurrentState();

    if (implementation == SwerveImplementation.WPILib) {
      odometry.update(Rotation2d.fromDegrees(getHeading()), flState, frState, blState, brState);
    } else {
      // Do the inverse of swerve math; rather than calculate swerve module states from chassis speeds, calculate chassis speeds from swerve module states.
      double temp[] = new double[3];
      if (SmartDashboard.getBoolean("Field Oriented", true)) {
        temp = SwerveMath.inverseSwerve(Constants.DriveConstants.wheelBase,
                                        Constants.DriveConstants.trackWidth, 
                                        getHeading(), 
                                        flState, frState, blState, brState);
      } else {
        temp = SwerveMath.inverseSwerve(Constants.DriveConstants.wheelBase,
                                        Constants.DriveConstants.trackWidth,
                                        flState, frState, blState, brState);
      }

      // Only forward and strafe are required
      double actualForward = temp[0];
      double actualStrafe = temp[1];

      // Period is 20 miliseconds since period is called every 20 miliseconds.
      double period = 20e-3;
      double deltaAngle = (getHeading() + gyroOffset) - pose.getRotation().getRadians();
      double deltaX = actualForward * period;
      double deltaY = actualStrafe * period;
      Pose2d newPose = pose.exp(new Twist2d(deltaX, deltaY, deltaAngle));
      pose = new Pose2d(newPose.getTranslation(), new Rotation2d(getHeading() + gyroOffset));
    }
  }

  /**
   * Updates SmartDashboard with information about the drivetrain pertinent to the operator.
  */
  public void updateSmartDashboard() {
    // Display the status of the odometry.
    SmartDashboard.putNumber("Pose X", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getTranslation().getX() : pose.getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getTranslation().getY() : pose.getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation (Radians)", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getRotation().getRadians() : pose.getRotation().getRadians());

    moduleFL.updateSmartDahsboard();
    moduleFR.updateSmartDahsboard();
    moduleBL.updateSmartDahsboard();
    moduleBR.updateSmartDahsboard();
   }

  @Override
  /**
   * Periodic method of the Drivetrain. Called every 20 miliseconds.
   */
  public void periodic() {
    updateOdometry();
    updateSmartDashboard();
  }

  /** 
   * Returns robot heading from the gyro in degrees.
  */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * (isGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Drives the robot.
   * @param forward   Desired forward speed (in m/s).
   * @param strafe    Desired strafe speed (in m/s).
   * @param rotation  Desired rotation speed (in rad/s).
   */
  public void drive(double forward, double strafe, double rotation) {
    System.out.println(forward + ", " + strafe + ", " + rotation);
    SwerveModuleState[] moduleStates;
    if (implementation == SwerveImplementation.WPILib) {
      moduleStates = getSwerveStates(forward, strafe, rotation);
    } else {
      if (SmartDashboard.getBoolean("Field Oriented", true)) {
        moduleStates = SwerveMath.calculateSwerve(forward, strafe, rotation, Math.toRadians(getHeading()), 
                                                Constants.DriveConstants.wheelBase, Constants.DriveConstants.trackWidth);
      } else {
        moduleStates = SwerveMath.calculateSwerve(forward, strafe, rotation,
                                                Constants.DriveConstants.wheelBase, Constants.DriveConstants.trackWidth);
      }
    }

    SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.DriveConstants.maxSpeed);
    SmartDashboard.putNumber("BL Swerve State Angle", moduleStates[2].angle.getRadians());
    SmartDashboard.putNumber("BL Selected Sensor Position", moduleBL.getSensorPosition());

    // Move the modules based on desired (normalized) speed, desired angle, max speed, drive modifier, and whether or not to reverse turning.
    moduleFL.move(moduleStates[0].speedMetersPerSecond, moduleStates[0].angle.getRadians(), 
                  Constants.DriveConstants.maxSpeed,  -Constants.DriveConstants.kDriveModifier, Constants.DriveConstants.reversedFL);
    moduleFR.move(moduleStates[1].speedMetersPerSecond, moduleStates[1].angle.getRadians(), 
                  Constants.DriveConstants.maxSpeed, Constants.DriveConstants.kDriveModifier, Constants.DriveConstants.reversedFR);
    moduleBL.move(moduleStates[2].speedMetersPerSecond, moduleStates[2].angle.getRadians(), 
                  Constants.DriveConstants.maxSpeed, -Constants.DriveConstants.kDriveModifier, Constants.DriveConstants.reversedBL);
    moduleBR.move(moduleStates[3].speedMetersPerSecond, moduleStates[3].angle.getRadians(), 
                  Constants.DriveConstants.maxSpeed, Constants.DriveConstants.kDriveModifier, Constants.DriveConstants.reversedBR);
  }

  /**
   * Constructs and returns a ChassisSpeeds objects using forward, strafe, and rotation values.
   * 
   * @param forward     The desired forward speed, in m/s.
   * @param strafe      The desired strafe speed, in m/s.
   * @param rotation    The desired rotation speed, in rad/s.
   * @return A ChassisSpeeds object.
  */
  private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
    ChassisSpeeds speeds;
    if (SmartDashboard.getBoolean("Field Oriented", true)) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()));
    } else {
      speeds = new ChassisSpeeds(forward, strafe, rotation);
    }
    return speeds;
  }

  /**
   * Constructs and returns four SwerveModuleState objects, one for each side, using forward, strafe, and rotation values.
   * 
   * @param forward     The desired forward speed, in m/s.
   * @param strafe      The desired strafe speed, in m/s.
   * @param rotation    The desired rotation speed, in rad/s.
   * @return A SwerveModuleState array, one for each side of the drivetrain (FL, FR, etc.).
  */
  private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
    return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, -rotation));
  }
}
