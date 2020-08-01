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

  public SwerveModule moduleFL;
  public SwerveModule moduleFR;
  public SwerveModule moduleBL;
  public SwerveModule moduleBR;

  private double speeds[] = {0.0, 0.0, 0.0, 0.0};

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
    moduleFL = new SwerveModule(MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontLeft), 
                                MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontLeft), 
                                Constants.DriveConstants.FL_GEAR_RATIO);
    moduleFR = new SwerveModule(MotorControllerFactory.createTalon(Constants.Ports.kDriveFrontRight), 
                                MotorControllerFactory.createTalon(Constants.Ports.kTurnFrontRight), 
                                Constants.DriveConstants.FR_GEAR_RATIO);
    moduleBL = new SwerveModule(MotorControllerFactory.createTalon(Constants.Ports.kDriveBackLeft), 
                                MotorControllerFactory.createTalon(Constants.Ports.kTurnBackLeft), 
                                Constants.DriveConstants.BL_GEAR_RATIO);
    moduleBR = new SwerveModule(MotorControllerFactory.createTalon(Constants.Ports.kDriveBackRight), 
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
      Translation2d locationFL = new Translation2d(-Constants.DriveConstants.trackWidth / 2, Constants.DriveConstants.wheelBase / 2);
      Translation2d locationFR = new Translation2d(Constants.DriveConstants.trackWidth / 2, Constants.DriveConstants.wheelBase / 2);
      Translation2d locationBL = new Translation2d(-Constants.DriveConstants.trackWidth, -Constants.DriveConstants.wheelBase / 2);
      Translation2d locationBR = new Translation2d(Constants.DriveConstants.trackWidth / 2, -Constants.DriveConstants.wheelBase / 2);

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
    // There are no encoders on the drive motor controllers so assume current speed = desired speed
    SwerveModuleState flState = new SwerveModuleState(speeds[0], 
                                                      new Rotation2d(moduleFL.getModuleAngle((int) Constants.DriveConstants.FL_TURN_ZERO, Constants.DriveConstants.FL_MAX_ANALOG)));
    SwerveModuleState frState = new SwerveModuleState(speeds[1], 
                                                      new Rotation2d(moduleFR.getModuleAngle((int) Constants.DriveConstants.FR_TURN_ZERO, Constants.DriveConstants.FR_MAX_ANALOG)));
    SwerveModuleState blState = new SwerveModuleState(speeds[2], 
                                                      new Rotation2d(moduleBL.getModuleAngle((int) Constants.DriveConstants.BL_TURN_ZERO, Constants.DriveConstants.BL_MAX_ANALOG)));
    SwerveModuleState brState = new SwerveModuleState(speeds[3], 
                                                      new Rotation2d(moduleBR.getModuleAngle((int) Constants.DriveConstants.BR_TURN_ZERO, Constants.DriveConstants.BR_MAX_ANALOG)));

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
    SmartDashboard.putNumber("FL Target Angle", moduleFL.targetAngle);
    SmartDashboard.putNumber("FR Target Angle", moduleFR.targetAngle);
    SmartDashboard.putNumber("BL Target Angle", moduleBL.targetAngle);
    SmartDashboard.putNumber("BR Target Angle", moduleBR.targetAngle);
    // Display the status of the odometry.
    SmartDashboard.putNumber("Pose X", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getTranslation().getX() : pose.getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getTranslation().getY() : pose.getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation (Radians)", (implementation == SwerveImplementation.WPILib) ? 
      odometry.getPoseMeters().getRotation().getRadians() : pose.getRotation().getRadians());
    
    // Display the position of the quadrature encoder.
    SmartDashboard.putNumber("FL Quadrature Position", moduleFL.getQuadraturePosition());
    SmartDashboard.putNumber("FR Quadrature Position", moduleFR.getQuadraturePosition());
    SmartDashboard.putNumber("BL Quadrature Position", moduleBL.getQuadraturePosition());
    SmartDashboard.putNumber("BR Quadrature Position", moduleBR.getQuadraturePosition());

    // Display the position of the analog encoder.
    SmartDashboard.putNumber("FL Analog Position", moduleFL.getAnalogPosition());
    SmartDashboard.putNumber("FR Analog Position", moduleFR.getAnalogPosition());
    SmartDashboard.putNumber("BL Analog Position", moduleBL.getAnalogPosition());
    SmartDashboard.putNumber("BR Analog Position", moduleBR.getAnalogPosition());

    // Display the position of the raw analog encoder.
    SmartDashboard.putNumber("FL Raw Analog Position", moduleFL.getAnalogPositionRaw());
    SmartDashboard.putNumber("FR Raw Analog Position", moduleFR.getAnalogPositionRaw());
    SmartDashboard.putNumber("BL Raw Analog Position", moduleBL.getAnalogPositionRaw());
    SmartDashboard.putNumber("BR Raw Analog Position", moduleBR.getAnalogPositionRaw());

    // Display the module angle as calculated using the absolute encoder.
    SmartDashboard.putNumber("FL Module Angle", moduleFL.getModuleAngle((int) Constants.DriveConstants.FL_TURN_ZERO, Constants.DriveConstants.FL_MAX_ANALOG));
    SmartDashboard.putNumber("FR Module Angle", moduleFR.getModuleAngle((int) Constants.DriveConstants.FR_TURN_ZERO, Constants.DriveConstants.FR_MAX_ANALOG));
    SmartDashboard.putNumber("BL Module Angle", moduleBL.getModuleAngle((int) Constants.DriveConstants.BL_TURN_ZERO, Constants.DriveConstants.BL_MAX_ANALOG));
    SmartDashboard.putNumber("BR Module Angle", moduleBR.getModuleAngle((int) Constants.DriveConstants.BR_TURN_ZERO, Constants.DriveConstants.BR_MAX_ANALOG));
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
    speeds = new double[]{moduleStates[0].speedMetersPerSecond, moduleStates[1].speedMetersPerSecond,
                            moduleStates[2].speedMetersPerSecond, moduleStates[3].speedMetersPerSecond};
    System.out.println(moduleStates[0].angle.getRadians());
    moduleFL.move(speeds[0], moduleStates[0].angle.getRadians(), Constants.DriveConstants.kDriveModifier, Constants.DriveConstants.reversedFL);
    moduleFR.move(speeds[1], moduleStates[1].angle.getRadians(), Constants.DriveConstants.kDriveModifier, Constants.DriveConstants.reversedFR);
    moduleBL.move(speeds[2], moduleStates[2].angle.getRadians(), Constants.DriveConstants.kDriveModifier, Constants.DriveConstants.reversedBL);
    moduleBR.move(speeds[3], moduleStates[3].angle.getRadians(), Constants.DriveConstants.kDriveModifier, Constants.DriveConstants.reversedBR);
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
   * @return A ChassisSpeeds object.
  */
  private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
    return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, strafe, rotation));
  }
}
